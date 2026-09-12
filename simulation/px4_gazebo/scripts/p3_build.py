"""Content-addressed P3 package build, preserving P2 receipts and mirrors."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from p2_build import sha, package_hash


def manifest(sim):
    source=sim/'ros2_ws/src/gwm_sensor_adapter'
    return {p.relative_to(source).as_posix():sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}


def build():
    sim=Path(__file__).resolve().parents[1]
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy'))).resolve()
    if Path.home().resolve() not in root.parents: raise ValueError('Linux HOME workspace required')
    files=manifest(sim); identity=package_hash(files)
    ws=root/'p3_ws'/identity; source=ws/'src/gwm_sensor_adapter'
    if source.exists():
        if any(sha(source/n)!=h for n,h in files.items()): raise ValueError('Changed P3 mirror')
    else:
        for n in files:
            destination=source/n; destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(sim/'ros2_ws/src/gwm_sensor_adapter'/n,destination)
    for command in (['colcon','build','--packages-select','gwm_sensor_adapter','--executor','sequential'],
                    ['colcon','test','--packages-select','gwm_sensor_adapter','--event-handlers','console_direct+'],
                    ['colcon','test-result','--verbose']):
        subprocess.run(command,cwd=ws,check=True)
    receipt=dict(package_hash=identity,source_files=files,workspace=str(ws),install=str(ws/'install'))
    (root/'state/p3-built.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('P3 built/tested '+identity)


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--build',action='store_true')
    if p.parse_args().build: build()
