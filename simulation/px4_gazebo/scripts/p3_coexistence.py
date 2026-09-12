"""Explicit P3 sensor session around the unchanged P2 controller process."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import time
import xml.etree.ElementTree as ET
from p1_runner import digest
from p2_build import package_hash
from p3_build import manifest


class SensorSession:
    def __init__(self,matrix):
        self.matrix=matrix
        self.children=[]

    def controller_config(self,sim):
        path=sim/'configs/p3_control.yaml'
        c=json.loads(path.read_text()); baseline=json.loads((sim/'configs/p2_control.yaml').read_text())
        expected=dict(baseline,world='gwm_p3_flight',model='x500_depth_71',simulation_profile='p3-depth-coexistence-v1')
        if c!=expected: raise ValueError('P3 must preserve every P2 flight setting')
        return path

    def identity(self,sim,root):
        matrix=json.loads((self.matrix/'summary.json').read_text())
        if matrix['status']!='passed' or len(matrix['trials'])!=6:
            raise ValueError('Complete passed ground matrix required before depth flight profile')
        first=Path(matrix['plane2_run'])
        ground=json.loads((first/'summary.json').read_text())
        if json.loads((first/'p3-evaluation.json').read_text())['status']!='passed': raise ValueError('plane2_not_passed')
        for p,h in ground['asset_hashes'].items():
            if digest(root/'upstream/PX4-Autopilot'/p)!=h: raise ValueError('Depth dynamics/model changed')
        receipt=json.loads((root/'state/p3-built.json').read_text())
        if package_hash(manifest(sim))!=receipt['package_hash']: raise ValueError('P3 package build mismatch')
        self.receipt=receipt
        return dict(ground_matrix=str(self.matrix),ground_matrix_sha256=digest(self.matrix/'summary.json'),
            sensor_package_hash=receipt['package_hash'],asset_hashes=ground['asset_hashes'],
            source_hashes={name:digest(sim/name) for name in ('scripts/p3_coexistence.py','scripts/run_p3_coexistence.sh',
                'configs/p3_sensors.yaml','configs/p3_fastdds.xml','configs/p3_sensor_bridge.yaml','configs/p3_validation.yaml',
                'validation/p3_source_probe.py','validation/collect_p3_coexistence.py')})

    def prepare(self,sim,root,run):
        self.sensor_dir=run/'sensors'; self.sensor_dir.mkdir()
        px4=root/'upstream/PX4-Autopilot'
        tree=ET.parse(px4/'Tools/simulation/gz/worlds/default.sdf')
        world=tree.getroot().find('world'); world.set('name','gwm_p3_flight')
        # One distant wall makes a measurable front surface. Closest visual and
        # collision point x=10: >=7.6m from the radius-2m + 0.4m rotor envelope.
        model=ET.SubElement(world,'model',name='distant_front_surface')
        ET.SubElement(model,'static').text='true'; ET.SubElement(model,'pose').text='10.05 0 4 0 0 0'
        link=ET.SubElement(model,'link',name='link')
        for kind in ('visual','collision'):
            geo=ET.SubElement(ET.SubElement(link,kind,name=kind),'geometry')
            ET.SubElement(ET.SubElement(geo,'box'),'size').text='.1 30 10'
        tree.write(run/'gwm_p3_flight.sdf',encoding='unicode')
        with (run/'rootfs/gz_env.sh').open('a') as f: f.write('\nexport PX4_GZ_WORLDS='+str(run)+'\n')
        for name in ('p3_sensors.yaml','p3_sensor_bridge.yaml','p3_fastdds.xml','p3_validation.yaml'):
            shutil.copyfile(sim/'configs'/name,self.sensor_dir/name)
        c=json.loads((self.sensor_dir/'p3_sensors.yaml').read_text())
        c.update(world='gwm_p3_flight',require_px4_state=True)
        (self.sensor_dir/'p3_sensors.yaml').write_text(json.dumps(c,indent=2)+'\n')
        (run/'p3-flight-profile.json').write_text(json.dumps(dict(world_sha256=digest(run/'gwm_p3_flight.sdf'),
            model='x500_depth_71',px4_airframe='4002_gz_x500_depth sources 4001_gz_x500',
            rendering_profile='p3-ogre2-server-WSLg-v1',native_sensor_workload_unchanged=True,
            native_camera_mass_kg=.061,base_vehicle_mass_kg=2+4*.016076923076923075,
            vehicle_with_camera_mass_kg=2+4*.016076923076923075+.061,
            horizontal_center_radius_m=2,rotor_enclosing_radius_m=.4,minimum_fixture_margin_m=7.6,
            controller_parameter_changes=False,sensor_driven_control=False),indent=2)+'\n')

    def start(self,sim,root,run,env,launch,topics):
        if any(t not in topics.splitlines() for t in ('/depth_camera','/camera_info')): raise ValueError('Owned depth topics absent')
        sensor_env=dict(env,GWM_P3_OWNED_NAMESPACE='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(self.sensor_dir/'p3_fastdds.xml'))
        self.children.append(launch('sensor-bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args',
            '-p','config_file:='+str(self.sensor_dir/'p3_sensor_bridge.yaml')],sensor_env))
        self.children.append(launch('sensor-source-probe',['/usr/bin/python3',str(sim/'validation/p3_source_probe.py'),str(self.sensor_dir)],env))
        self.children.append(launch('sensor-adapter',[str(Path(self.receipt['install'])/'gwm_sensor_adapter/lib/gwm_sensor_adapter/p3_sensor'),
            '--run-dir',str(self.sensor_dir),'--config',str(self.sensor_dir/'p3_sensors.yaml')],sensor_env))
        start=time.monotonic()
        while not (self.sensor_dir/'sensor-ready.json').exists():
            if any(p.poll() is not None for p in self.children): raise RuntimeError('sensor_readiness_process_exit')
            if time.monotonic()-start>75: raise TimeoutError('sensor_readiness_timeout')
            time.sleep(.1)
        if shutil.disk_usage(root).free<20*1024**3: raise ValueError('sensor_recording_disk_reserve')

    def stop(self,run):
        for p in reversed(self.children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=15)

    def finish(self,run):
        try: self.stop(run)
        except Exception as exc: (run/'sensor-cleanup-error.txt').write_text(str(exc))
        if hasattr(self,'sensor_dir'):
            artifacts={p.name:dict(bytes=p.stat().st_size,sha256=digest(p)) for p in self.sensor_dir.iterdir() if p.is_file()}
            (run/'sensor-artifacts.json').write_text(json.dumps(artifacts,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--run',action='store_true',required=True)
    p.add_argument('--observe',action='store_true'); p.add_argument('--allow-simulated-flight',action='store_true')
    p.add_argument('--ground-matrix',type=Path,required=True)
    args=p.parse_args(); args.ground_diagnostic=False; args.native_wait_probe=False; args.diagnostic=False; args.headless=True
    if args.allow_simulated_flight:
        root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
        connection=json.loads((root/'state/p3-connection.json').read_text())
        previous=root/'runs'/connection['run_id']
        sensor=json.loads((previous/'p3-coexistence-evaluation.json').read_text())
        control=json.loads((previous/'p2-offline-evaluation.json').read_text())
        if sensor['status']!='passed' or control['recording_integrity']!='passed':
            raise ValueError('Independent sensor and control connection checks must pass before flight')
    from p2_runner import trial
    raise SystemExit(trial(args,SensorSession(args.ground_matrix)))
