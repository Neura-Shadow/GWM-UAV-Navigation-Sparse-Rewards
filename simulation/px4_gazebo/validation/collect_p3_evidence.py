"""Offline P3 acceptance. Analytic renderer geometry never imports adapter code."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def acquisition_ns(record, required=False):
    """Keep native image/source identity; only legacy records need conversion."""
    if 'acquisition_sim_ns' in record:
        value=record['acquisition_sim_ns']
        if type(value) is not int or not 0 <= value < 2**63: raise ValueError('invalid_native_acquisition_ns')
        return value
    if required: raise ValueError('missing_native_acquisition_ns')
    return round(record['acquisition_sim_s']*1e9)


def rotation(q):
    x,y,z,w=[q.get(k,0.) for k in ('x','y','z','w')]
    if abs(x*x+y*y+z*z+w*w-1)>1e-6: raise ValueError('truth_quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def pose(run,name):
    data=json.loads((run/(name+'.json')).read_text())
    model=next(p for p in data['pose'] if p['name']=='x500_depth_71')
    return np.array([model['position'].get(k,0.) for k in ('x','y','z')]),rotation(model['orientation'])


def expected_depth(u,v,info,position,attitude,fixture):
    """Ray/OBB slab intersections in world coordinates; parameter is optical Z."""
    p=info['p']
    body_rays=np.stack([np.ones_like(u),-(u-p[2])/p[0],-(v-p[6])/p[5]],axis=-1)
    # Fixed three-component contractions do not need a BLAS/OpenMP runtime.
    # This also permits pure tests beside the repository's Windows PyTorch.
    rays=np.einsum('...j,ij->...i',body_rays,attitude,optimize=False)
    origin=position+np.sum(attitude*np.array(fixture['optical_origin_model_m']),axis=1)
    result=np.full(u.shape,np.inf)
    labels=np.full(u.shape,-1,dtype=int)
    for index,box in enumerate(fixture['boxes']):
        yaw=box['yaw_rad']; c,s=math.cos(yaw),math.sin(yaw)
        rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        local_origin=np.einsum('i,ij->j',origin-np.array(box['center']),rot,optimize=False)
        local_rays=np.einsum('...i,ij->...j',rays,rot,optimize=False)
        half=np.array(box['size'])/2
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-half-local_origin)/local_rays; b=(half-local_origin)/local_rays
        lo=np.minimum(a,b).max(axis=-1); hi=np.maximum(a,b).min(axis=-1)
        hit=(hi>=np.maximum(lo,0)) & (lo>0) & (lo<result)
        result[hit]=lo[hit]; labels[hit]=index
    with np.errstate(divide='ignore',invalid='ignore'): floor=-origin[2]/rays[...,2]
    hit=(floor>0)&(rays[...,2]<0)&(floor<result)
    result[hit]=floor[hit]; labels[hit]=-2
    return result,labels,np.sqrt(np.sum(body_rays*body_rays,axis=-1))


def evaluate(run):
    summary=json.loads((run/'summary.json').read_text())
    out=dict(schema_version=1,run_id=run.name,status='failed',failures=[],evaluator_sha256=digest(Path(__file__)))
    revised=summary.get('frozen_inputs',{}).get('sample_evidence_contract') == 'p3-sample-evidence-v2'
    if revised:
        from p3_provenance import require_finalized, frozen_inputs
        require_finalized(run,frozen_inputs(Path(__file__).resolve().parents[1]))
        out.update(schema_version=2,sample_evidence_contract='p3-sample-evidence-v2',frozen_inputs=summary['frozen_inputs'])
    def check(condition,reason):
        if not condition: out['failures'].append(reason)
    check(summary['status']=='collected','run_not_collected')
    for name,record in summary['artifacts'].items():
        check((run/name).stat().st_size==record['bytes'] and digest(run/name)==record['sha256'],'artifact:'+name)
    if summary['status']!='collected': return out
    limits=json.loads((run/'p3_validation.yaml').read_text())
    events=[json.loads(line) for line in (run/'sensor-events.jsonl').read_text().splitlines()]
    rows=[json.loads(line) for line in (run/'depth-index.jsonl').read_text().splitlines()]
    result=json.loads((run/'sensor-result.json').read_text())
    begin,end=summary['window']['start_sim_s'],summary['window']['end_sim_s']
    begin_ns,end_ns=round(begin*1e9),round(end*1e9)
    window=[r for r in rows if begin_ns<=acquisition_ns(r,revised)<=end_ns]
    observations=[e for e in events if e['kind']=='observation' and begin_ns<=acquisition_ns(e['image'],revised)<=end_ns]
    health=[e for e in events if e['kind']=='health' and begin<=e['sim_s']<=end]
    check(result['status']=='complete','recorder_completion')
    check(len(rows)==result['received']==result['recorder']['written']==result['recorder']['accepted'],'recording_coverage')
    check(result['recorder']['overflow']==0,'recorder_overflow')
    out['recording']=result
    out['received_frames']=len(rows)
    out['raw_bytes']=(run/'depth.bin').stat().st_size
    out['calibration']=next(e for e in events if e['kind']=='calibration')
    info=out['calibration']['info']
    if summary['case']=='interruption':
        interruption=summary['interruption_wall_s']
        transitions=[e for e in events if e['kind']=='health' and e['wall_s']>=interruption and e['status'] in ('stale','unavailable')]
        check(bool(transitions),'interruption_no_transition')
        out['detection_wall_s']=transitions[0]['wall_s']-interruption if transitions else None
        check(bool(transitions) and out['detection_wall_s']<=limits['interruption_detect_wall_s'],'interruption_late')
        check(not [e for e in observations if e['processing_wall_s']>interruption+limits['interruption_detect_wall_s']],'reused_fresh_observation')
        out['status']='passed_expected_failure_diagnostic' if not out['failures'] else 'failed'
        return out
    check(len(window)>=2 and len(observations)>=2,'missing_measurements')
    if len(window)<2 or len(observations)<2: return out
    stamps_ns=np.array([acquisition_ns(r,revised) for r in window],dtype=np.int64)
    gaps_ns=np.diff(stamps_ns); gaps=gaps_ns/1e9
    stamps=stamps_ns/1e9
    rate=(len(window)-1)*1e9/int(stamps_ns[-1]-stamps_ns[0])
    obs_stamps=np.array([acquisition_ns(e['image'],revised) for e in observations],dtype=np.int64)
    processed_rate=(len(observations)-1)*1e9/int(obs_stamps[-1]-obs_stamps[0])
    out['timing']=dict(window_sim_s=end-begin,frames=len(window),delivered_hz=rate,
        source_gap_max_sim_s=float(gaps.max()),processed_hz=processed_rate,observations=len(observations),
        observation_age_max_sim_s=max(e['source_age_sim_s'] for e in observations),
        processing_max_wall_s=max(e['processing_return_wall_s']-e['processing_wall_s'] for e in observations))
    check(stamps[0]-begin<=limits['source_gap_sim_s'] and end-stamps[-1]<=limits['source_gap_sim_s'],'window_coverage')
    check(np.all(gaps>0) and gaps.max()<=limits['source_gap_sim_s'],'source_gaps')
    check(rate>=limits['delivered_hz_min'],'delivered_rate')
    check(processed_rate>=limits['processed_hz_min'],'processed_rate')
    check(out['timing']['observation_age_max_sim_s']<=limits['observation_age_sim_s'],'observation_age')
    check(health and all(e['status']=='fresh' for e in health),'health_not_fresh')
    check(not [e for e in events if e['kind']=='error' and begin<=e.get('sim_s',-1)<=end],'window_errors')
    position,attitude=pose(run,'truth-start'); end_position,end_attitude=pose(run,'truth-end')
    drift=float(np.linalg.norm(position-end_position))
    rot_drift=float(math.acos(np.clip((np.trace(attitude.T@end_attitude)-1)/2,-1,1)))
    check(drift<=limits['geometry_pose_translation_drift_m'] and rot_drift<=limits['geometry_pose_rotation_drift_rad'],'unsettled_pose')
    out['pose']=dict(model_world_m=position.tolist(),optical_world_m=(position+attitude@np.array([.13233,0,.26078])).tolist(),
                     translation_drift_m=drift,rotation_drift_rad=rot_drift)
    u0,v0,u1,v1=limits['plane_roi_uv']; v,u=np.mgrid[v0:v1,u0:u1]
    expected,labels,ray_norm=expected_depth(u,v,info,position,attitude,summary['fixture'])
    observations_by_id={e['image']['id']:e for e in observations}
    valid_min=1.; median_max=p95_max=0.; range_error=[]; unique=set(); coordinates=0; coord_error=0.
    counts=dict(nan=0,positive_infinity=0,negative_infinity=0,finite_below_range=0,finite_above_range=0)
    offset=0
    per_frame=[]
    asymmetric={name:[] for name in ('left_high','right_low')}
    with (run/'depth.bin').open('rb') as data:
        for index,row in enumerate(rows):
            check(row['id']==index+1 and row['offset']==offset,'raw_index_continuity')
            raw=data.read(row['bytes']); offset+=len(raw)
            check(hashlib.sha256(raw).hexdigest()==row['sha256'],'raw_frame_hash')
            if not begin_ns<=acquisition_ns(row,revised)<=end_ns: continue
            check((row['width'],row['height'],row['encoding'],row['step'],row['is_bigendian'])==(640,480,'32FC1',2560,0),'actual_layout')
            image=np.frombuffer(raw,dtype='<f4').reshape(480,640)
            if summary['case']=='asymmetric':
                # Fixed interior regions, declared before the asymmetric run.
                for name,(ax,ay,bx,by),target in (('left_high',(195,55,205,65),3.),
                                                ('right_low',(455,115,465,125),2.)):
                    region=image[ay:by,ax:bx]
                    check(np.isfinite(region).all() and np.max(np.abs(region-target))<.05,'asymmetric:'+name)
                    asymmetric[name].append(float(np.median(region)))
            roi=image[v0:v1,u0:u1]
            valid=np.isfinite(roi)&(roi>=.2)&(roi<19.1)
            valid_min=min(valid_min,float(valid.mean()))
            counts['nan']+=int(np.isnan(image).sum()); counts['positive_infinity']+=int(np.isposinf(image).sum())
            counts['negative_infinity']+=int(np.isneginf(image).sum())
            counts['finite_below_range']+=int((np.isfinite(image)&(image<.2)).sum())
            counts['finite_above_range']+=int((np.isfinite(image)&(image>=19.1)).sum())
            unique.add(row['sha256'])
            if valid.any():
                errors=np.abs(roi[valid]-expected[valid]); med=float(np.median(errors)); p95=float(np.percentile(errors,95))
                median_max=max(median_max,med); p95_max=max(p95_max,p95)
                range_error.append(float(np.median(np.abs(roi[valid]-expected[valid]*ray_norm[valid]))))
                per_frame.append(dict(id=row['id'],median_measured_m=float(np.median(roi[valid])),median_error_m=med,p95_error_m=p95,valid_fraction=float(valid.mean())))
            obs=observations_by_id.get(row['id'])
            if obs:
                for sample in obs['samples']:
                    d=image[sample['v'],sample['u']]
                    if np.isfinite(d) and .2<=d<19.1:
                        # Independent direct P->body formula, not production backprojection.
                        p=info['p']; xyz=[float(d)+.13233,-(sample['u']-p[2])*float(d)/p[0],
                                        -(sample['v']-p[6])*float(d)/p[5]+.02078]
                        err=float(np.max(np.abs(np.array(xyz)-sample['body_flu_m'])))
                        coord_error=max(coord_error,err); coordinates+=1
                    else: check(sample['depth_m'] is None and sample['body_flu_m'] is None,'unknown_filled')
        check(data.read(1)==b'','raw_trailing_data')
    # Stationary geometry can produce byte-identical frames. Dynamic rendering
    # is separately established by the post-window plane4 challenge.
    check(coordinates>0 and coord_error<1e-6,'coordinate_transform')
    out['geometry']=dict(roi_uv=limits['plane_roi_uv'],pixels_per_frame=int(expected.size),valid_fraction_min=valid_min,
        expected_median_m=float(np.median(expected)),measured_median_m=float(np.median([p['median_measured_m'] for p in per_frame])) if per_frame else None,
        median_absolute_error_max_m=median_max,p95_absolute_error_max_m=p95_max,
        euclidean_range_hypothesis_median_error_m=float(np.median(range_error)) if range_error else None,
        coordinate_samples=coordinates,coordinate_error_max_m=coord_error,unique_raw_frames=len(unique),invalid_counts=counts)
    if (run/'gazebo-source-headers.jsonl').exists():
        source=[json.loads(line) for line in (run/'gazebo-source-headers.jsonl').read_text().splitlines()]
        source=[s for s in source if begin_ns<=acquisition_ns(s,revised)<=end_ns]
        source_times=np.array([acquisition_ns(s,revised) for s in source],dtype=np.int64)
        out['source_probe']=dict(frames=len(source),source_gap_max_sim_s=float(np.diff(source_times).max()/1e9),
            gazebo_frames_missing_in_ros=len(set(source_times)-set(stamps_ns)),
            source_sequence_first=source[0]['header_data'],source_sequence_last=source[-1]['header_data'])
        check(out['source_probe']['gazebo_frames_missing_in_ros']==0,'source_to_recording_loss')
        seq=[int(s['header_data']['seq'][0]) for s in source]
        check(all(b-a==1 for a,b in zip(seq,seq[1:])),'source_probe_incomplete')
    else: check(False,'source_recording_coverage_unproven')
    if summary['case']=='plane4':
        challenge=summary['render_challenge_wall_s']
        after=[e for e in events if e['kind']=='observation' and e['processing_wall_s']>challenge+1]
        center=[s['depth_m'] for e in after for s in e['samples'] if s['u']==320 and s['v']==160]
        check(center and all(d is not None and abs(d-5)<.05 for d in center),'render_challenge')
        out['render_challenge']=dict(after_expected_m=5,after_median_m=float(np.median(center)) if center else None,
                                     observations=len(center),excluded_from_measurement_window=True)
    if summary['case']=='out_of_range':
        check(valid_min==0 and counts['positive_infinity']>0,'no_return_semantics')
    else:
        check(valid_min>=limits['valid_fraction_min'],'plane_valid_fraction')
        check(median_max<=max(limits['median_error_m'],limits['median_error_fraction']*float(np.median(expected))),'median_depth_error')
        check(p95_max<=max(limits['p95_error_m'],limits['p95_error_fraction']*float(np.median(expected))),'p95_depth_error')
        if summary['case']=='oblique': check(np.median(range_error)>.1 and median_max<.05,'Z_vs_ray_range_not_distinguished')
        if summary['case']=='asymmetric':
            out['asymmetric']=dict(left_high_m=float(np.median(asymmetric['left_high'])),
                right_low_m=float(np.median(asymmetric['right_low'])),
                left_high_roi_uv=[195,55,205,65],right_low_roi_uv=[455,115,465,125],
                verified_axes='image_right_to_body_negative_Y; image_up_to_body_positive_Z; positive_forward_X',
                optical_to_base_translation_m=[.13233,0,.02078])
    out['status']='passed' if not out['failures'] else 'failed'
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('run',type=Path); args=p.parse_args()
    try: result=evaluate(args.run)
    except Exception as exc: result=dict(schema_version=2,run_id=args.run.name,status='failed',failure=str(exc),
        sample_evidence_contract='p3-sample-evidence-v2',evaluator_sha256=digest(Path(__file__)))
    with (args.run/'p3-evaluation.json').open('x') as stream: stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if result['status'].startswith('passed') else 1)
