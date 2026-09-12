"""Owned ground-only rendering/calibration supervisor; no PX4 flight API."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from p1_contract import require_gates
from p2_build import sha, package_hash
from p3_build import manifest


def box(world, name, front, size, yaw=0):
    # front is the visible negative-local-X face, not the box center.
    c,s=math.cos(yaw),math.sin(yaw)
    center=[front[0]+c*size[0]/2, front[1]+s*size[0]/2,front[2]]
    model=ET.SubElement(world,'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,center+[0,0,yaw]))
    link=ET.SubElement(model,'link',name='fixture_link')
    for kind in ('visual','collision'):
        part=ET.SubElement(link,kind,name=kind)
        geometry=ET.SubElement(part,'geometry')
        ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,size))
    return dict(name=name,center=center,size=size,yaw_rad=yaw,front=front)


def make_world(px4, run, fixture, case):
    source=px4/'Tools/simulation/gz/worlds/default.sdf'
    tree=ET.parse(source); world=tree.getroot().find('world')
    world.set('name','gwm_p3_ground')
    include=ET.SubElement(world,'include')
    ET.SubElement(include,'uri').text='model://x500_depth'
    ET.SubElement(include,'name').text='x500_depth_71'
    ET.SubElement(include,'pose').text='0 0 .1 0 0 0'
    fixtures=[box(world,'front_plane',[fixture['distance_m']+.13233,0,4],[.1,20,20],fixture['yaw_rad'])]
    if case=='asymmetric':
        fixtures += [box(world,'right_low',[2.13233,-.65,.8],[.1,.5,.4]),
                     box(world,'left_high',[3.13233,.85,1.5],[.1,.5,.6])]
    target=run/'gwm_p3_ground.sdf'; tree.write(target,encoding='unicode')
    return dict(source_world_sha256=sha(source),generated_world_sha256=sha(target),boxes=fixtures,
                ground_plane_z=0,world='gwm_p3_ground',model='x500_depth_71',
                optical_origin_model_m=[.13233,0,.26078],base_origin_model_m=[0,0,.24])


def trial(args):
    require_gates(os.environ)
    interfaces=json.loads(subprocess.check_output(['ip','-j','link'],text=True))
    if os.getpid()!=1 or {i['ifname'] for i in interfaces}!={'lo'}: raise ValueError('Owned namespace required')
    sim=Path(__file__).resolve().parents[1]
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    from gwm_sensor_adapter.contracts import load_config
    config=load_config(sim/'configs/p3_sensors.yaml')
    validation=json.loads((sim/'configs/p3_validation.yaml').read_text())
    receipt=json.loads((root/'state/p3-built.json').read_text())
    if package_hash(manifest(sim))!=receipt['package_hash']: raise ValueError('P3 build mismatch')
    for n,h in receipt['source_files'].items():
        if sha(Path(receipt['workspace'])/'src/gwm_sensor_adapter'/n)!=h: raise ValueError('Changed mirror')
    case='plane4' if args.case=='interruption' else args.case
    if case not in validation['fixtures']: raise ValueError('Unknown case')
    run=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-p3-'+args.case+'-'+uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    print('P3 evidence: '+str(run),flush=True)
    px4=root/'upstream/PX4-Autopilot'
    models=px4/'Tools/simulation/gz/models'
    summary=dict(schema_version=1,run_id=run.name,case=args.case,status='incomplete',failure=None,
        project_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=sim,text=True).strip(),
        profile=config['profile'],package_hash=receipt['package_hash'],
        purpose='expected_failure_ground_interruption' if args.case=='interruption' else 'ground_calibration',
        rendering='ogre2_server_only_WSLg_display_native_sensors',control_owner='none',flight_commands=0,
        isolation='private user/mount/PID/network namespace, loopback only, exclusive P1/P2/P3 lock',
        p2_accepted_manifest_sha256=sha(sim.parents[1]/'docs/evidence/v3_sim_p2_summary.json'),
        sources={},processes=[])
    for name in ('p3_sensors.yaml','p3_sensor_bridge.yaml','p3_validation.yaml','p3_fastdds.xml'):
        shutil.copyfile(sim/'configs'/name,run/name)
    summary['source_files']={p.relative_to(sim).as_posix():sha(p) for folder in ('scripts','configs','validation','ros2_ws/src/gwm_sensor_adapter')
        for p in (sim/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    assets=[p for folder in ('x500_depth','OakD-Lite','x500','x500_base') for p in (models/folder).rglob('*') if p.is_file()]
    summary['asset_hashes']={p.relative_to(px4).as_posix():sha(p) for p in assets}
    summary['dependency_revisions']={str(p.relative_to(root)):subprocess.check_output(['git','rev-parse','HEAD'],cwd=p,text=True).strip()
        for p in (px4,px4/'Tools/simulation/gz',root/'ros_ws/src/px4_msgs')}
    summary['disk_free_before_bytes']=shutil.disk_usage(root).free
    processes=[]; streams=[]
    env={k:v for k,v in os.environ.items() if k not in ('FASTDDS_BUILTIN_TRANSPORTS','SKIP_DEFAULT_XML','RMW_FASTRTPS_PUBLICATION_MODE')}
    env.update(ROS_DOMAIN_ID='71',GZ_PARTITION=run.name,GZ_IP='127.0.0.1',GZ_DISTRO='harmonic',
        GZ_SIM_RESOURCE_PATH=str(models),SDF_PATH=str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(px4/'build/px4_sitl_default_linux/src/modules/simulation/gz_plugins')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH',''),
        GZ_SIM_SERVER_CONFIG_PATH=str(px4/'src/modules/simulation/gz_bridge/server.config'),
        GWM_P3_OWNED_NAMESPACE='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    summary['process_environment']={k:env.get(k) for k in ('ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','DISPLAY','GZ_SIM_SERVER_CONFIG_PATH',
        'RMW_IMPLEMENTATION','FASTDDS_BUILTIN_TRANSPORTS','SKIP_DEFAULT_XML','RMW_FASTRTPS_PUBLICATION_MODE')}
    def save(): (run/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    def launch(name,command):
        stream=(run/(name+'.log')).open('w'); streams.append(stream)
        process_env=dict(env)
        if name in ('sensor-bridge','sensor-adapter'):
            process_env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(run/'p3_fastdds.xml')
        p=subprocess.Popen(command,cwd=run,env=process_env,stdout=stream,stderr=stream,start_new_session=True)
        processes.append((name,p)); summary['processes'].append(dict(name=name,pid=p.pid,command=command))
        summary['processes'][-1]['sensor_transport_xml']=process_env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
        return p
    def truth(name):
        raw=subprocess.check_output(['gz','topic','-e','-n','1','--json-output','-t','/world/gwm_p3_ground/dynamic_pose/info'],env=env,text=True,timeout=10)
        (run/(name+'.json')).write_text(raw)
        return json.loads(raw)
    start=time.monotonic()
    try:
        save()
        if summary['disk_free_before_bytes']<validation['disk_reserve_bytes']: raise ValueError('disk_preflight')
        summary['fixture']=make_world(px4,run,validation['fixtures'][case],case)
        resolve=subprocess.run(['gz','sdf','-p',str(models/'x500_depth/model.sdf')],env=env,text=True,capture_output=True,check=True)
        (run/'resolved-model.sdf').write_text(resolve.stdout); (run/'resolved-model.log').write_text(resolve.stderr)
        summary['resolved_model_sha256']=sha(run/'resolved-model.sdf')
        launch('gazebo',['gz','sim','-r','-s','--render-engine','ogre2','-v','4',str(run/'gwm_p3_ground.sdf')])
        launch('source-probe',['/usr/bin/python3',str(sim/'validation/p3_source_probe.py'),str(run)])
        clock=(sim/'configs/p2_clock_bridge.yaml').read_text().replace('/world/default/clock','/world/gwm_p3_ground/clock')
        (run/'clock-bridge.yaml').write_text(clock)
        launch('clock-bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(run/'clock-bridge.yaml')])
        bridge=launch('sensor-bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(run/'p3_sensor_bridge.yaml')])
        sensor=launch('sensor-adapter',[str(Path(receipt['install'])/'gwm_sensor_adapter/lib/gwm_sensor_adapter/p3_sensor'),
            '--run-dir',str(run),'--config',str(run/'p3_sensors.yaml')])
        ready=run/'sensor-ready.json'
        while not ready.exists():
            if time.monotonic()-start>validation['startup_wall_s']: raise TimeoutError('sensor_readiness_timeout')
            for name,p in processes:
                if p.poll() is not None: raise RuntimeError('owned_process_exited:'+name)
            time.sleep(.1)
        topics=subprocess.check_output(['gz','topic','-l'],env=env,text=True,timeout=10)
        (run/'gazebo-topics.txt').write_text(topics)
        for topic in ('/depth_camera','/camera_info'):
            if topic not in topics.splitlines(): raise ValueError('Owned depth topic absent')
        (run/'ros-image-qos.txt').write_text(subprocess.check_output(['ros2','topic','info','-v',config['image_topic']],env=env,text=True,timeout=10))
        initial=truth('truth-start')
        stamp=initial['header']['stamp']; begin=int(stamp.get('sec',0))+int(stamp.get('nsec',0))/1e9
        summary['window']=dict(start_sim_s=begin,end_sim_s=begin+validation['ground_window_sim_s'])
        if args.case=='interruption':
            summary['interruption_wall_s']=time.monotonic()
            os.killpg(bridge.pid,signal.SIGTERM); bridge.wait(timeout=5)
            summary['window']['end_sim_s']=begin+3
        save()
        last_sim=0
        pending=''
        with (run/'sensor-events.jsonl').open() as events:
            while last_sim<summary['window']['end_sim_s']:
                pending+=events.read()
                lines=pending.split('\n'); pending=lines.pop()
                for line in lines:
                    e=json.loads(line)
                    if e['kind']=='health': last_sim=e['sim_s']
                if time.monotonic()-start>validation['ground_wall_deadline_s']: raise TimeoutError('ground_wall_deadline')
                if sensor.poll() is not None: raise RuntimeError('sensor_exited')
                time.sleep(.05)
        truth('truth-end')
        if args.case=='plane4':
            # Separate ground render challenge AFTER the fixed geometry window.
            summary['render_challenge_wall_s']=time.monotonic()
            response=subprocess.check_output(['gz','service','-s','/world/gwm_p3_ground/set_pose',
                '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','2000',
                '--req','name: "front_plane", position: {x: 5.18233, y: 0, z: 4}, orientation: {w: 1}'],
                env=env,text=True,timeout=5)
            summary['render_challenge_response']=response
            time.sleep(2)
        os.killpg(sensor.pid,signal.SIGTERM); sensor.wait(timeout=15)
        if sensor.returncode!=0: raise RuntimeError('sensor_failed')
        summary['status']='collected'
    except (Exception,KeyboardInterrupt) as exc:
        summary.update(status='failed',failure=str(exc) or 'interrupted')
    finally:
        for name,p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try: p.wait(timeout=15 if name=='sensor-adapter' else 5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL); p.wait(timeout=5)
            next(r for r in summary['processes'] if r['pid']==p.pid)['exit_code']=p.returncode
        for s in streams: s.close()
        rendering_log=Path.home()/'.gz/rendering/ogre2.log'
        if rendering_log.exists(): shutil.copyfile(rendering_log,run/'ogre2.log')
        summary['cleanup']='owned groups stopped; PID namespace exit reaps remaining descendants'
        summary['wall_duration_s']=time.monotonic()-start
        summary['artifacts']={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in run.iterdir() if p.is_file() and p.name!='summary.json'}
        save()
    print(json.dumps(dict(run_id=run.name,status=summary['status'],failure=summary['failure'])),flush=True)
    return 0 if summary['status']=='collected' else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--run',action='store_true',required=True)
    parser.add_argument('--case',required=True)
    raise SystemExit(trial(parser.parse_args()))
