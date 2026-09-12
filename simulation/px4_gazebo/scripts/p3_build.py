"""Content-addressed P3 package build, preserving P2 receipts and mirrors."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from p2_build import sha, package_hash,package_manifest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation'))
from p3_provenance import tree_manifest,atomic_json,strict_json


def manifest(sim):
    source=sim/'ros2_ws/src/gwm_sensor_adapter'
    return {p.relative_to(source).as_posix():sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}


def build():
    sim=Path(__file__).resolve().parents[1]
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy'))).resolve()
    if Path.home().resolve() not in root.parents: raise ValueError('Linux HOME workspace required')
    files=manifest(sim); identity=package_hash(files)
    controller=strict_json(root/'state/p2-built.json')
    if controller['package_hash']!=package_hash(package_manifest(sim)):
        raise ValueError('Build final controller before sensor package')
    import gwm_px4_control.sample_identity as shared
    expected=next(Path(controller['install']).rglob('site-packages/gwm_px4_control/sample_identity.py'))
    if Path(shared.__file__).resolve()!=expected.resolve(): raise ValueError('Sensor build controller overlay mismatch')
    ws=root/'p3_ws'/identity; source=ws/'src/gwm_sensor_adapter'
    if source.exists():
        if tree_manifest(source)!=files: raise ValueError('Changed P3 mirror')
    else:
        for n in files:
            destination=source/n; destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(sim/'ros2_ws/src/gwm_sensor_adapter'/n,destination)
    for command in (['colcon','build','--packages-select','gwm_sensor_adapter','--executor','sequential'],
                    ['colcon','test','--packages-select','gwm_sensor_adapter','--event-handlers','console_direct+'],
                    ['colcon','test-result','--verbose']):
        subprocess.run(command,cwd=ws,check=True)
    receipt=dict(package_hash=identity,source_files=files,workspace=str(ws),install=str(ws/'install'),
                 dependency_controller_package_hash=controller['package_hash'])
    installed=next((ws/'install').rglob('site-packages/gwm_sensor_adapter/__init__.py')).parent
    expected=tree_manifest(source/'gwm_sensor_adapter')
    if tree_manifest(installed)!=expected: raise ValueError('Installed P3 sources differ from mirror')
    receipt['installed_source_files']=expected
    atomic_json(root/'state/p3-built.json',receipt)
    print('P3 built/tested '+identity)


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--build',action='store_true')
    if p.parse_args().build: build()
