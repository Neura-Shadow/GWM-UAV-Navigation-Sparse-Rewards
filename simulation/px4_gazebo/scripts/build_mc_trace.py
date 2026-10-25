"""Build an isolated validation-only PX4 overlay; never edit the pinned checkout."""
import argparse
import ctypes
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM/'validation'))
from p3_provenance import atomic_json, digest, strict_json, tree_manifest

CPP = 'src/modules/mc_pos_control/MulticopterPositionControl.cpp'
CONTROL = 'src/modules/mc_pos_control/PositionControl/PositionControl.cpp'
CMAKE = 'src/modules/mc_pos_control/CMakeLists.txt'


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Pinned source anchor is not unique: '+repr(old))
    return text.replace(old, new, 1)


def instrument(original, name):
    text = original
    if name == CPP:
        text = replace_once(text, '#include <float.h>', '#include "GwmMcTrace.hpp"\n#include <float.h>')
        text = replace_once(text, '\tperf_free(_cycle_perf);', '\tgwm_trace::recorder.close();\n\tperf_free(_cycle_perf);')
        text = replace_once(text, 'bool MulticopterPositionControl::init()\n{', 'bool MulticopterPositionControl::init()\n{\n\tgwm_trace::recorder.init();')
        text = replace_once(text, 'if (_local_pos_sub.update(&vehicle_local_position)) {',
            'if (_local_pos_sub.update(&vehicle_local_position)) {\n\t\tgwm_trace::recorder.begin(vehicle_local_position, _local_pos_sub.get_last_generation(), hrt_absolute_time());')
        text = replace_once(text, '\t\tadjustSetpointForEKFResets(vehicle_local_position, _setpoint);',
            '\t\tgwm_trace::recorder.context(_trajectory_setpoint_sub.get_last_generation(),\n'
            '\t\t\tuint32_t(_vehicle_control_mode.flag_multicopter_position_control_enabled) |\n'
            '\t\t\t(uint32_t(_vehicle_control_mode.flag_armed) << 1) |\n'
            '\t\t\t(uint32_t(_vehicle_control_mode.flag_control_offboard_enabled) << 2));\n'
            '\t\tadjustSetpointForEKFResets(vehicle_local_position, _setpoint);')
        for token, path in (('_control.setInputSetpoint(_setpoint);', 1),
                            ('_control.setInputSetpoint(_last_valid_setpoint);', 2),
                            ('_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));', 3)):
            text = replace_once(text, token, f'gwm_trace::recorder.path({path});\n\t\t\t'+token)
        text = replace_once(text, '\t\t\t_local_pos_sp_pub.publish(local_pos_sp);',
            '\t\t\t_local_pos_sp_pub.publish(local_pos_sp);\n\t\t\tgwm_trace::recorder.output(local_pos_sp);')
        text = replace_once(text, '\tperf_end(_cycle_perf);', '\tgwm_trace::recorder.end(hrt_absolute_time());\n\tperf_end(_cycle_perf);')
    elif name == CONTROL:
        text = replace_once(text, '#include "PositionControl.hpp"', '#include "PositionControl.hpp"\n#include "../GwmMcTrace.hpp"')
        text = replace_once(text, '\t_vel_dot = states.acceleration;', '\t_vel_dot = states.acceleration;\n\tgwm_trace::recorder.state(_yaw);')
        text = replace_once(text, '\t_yawspeed_sp = setpoint.yawspeed;', '\t_yawspeed_sp = setpoint.yawspeed;\n\tgwm_trace::recorder.input(setpoint);')
        text = replace_once(text, '\tbool valid = _inputValid();', '\tgwm_trace::recorder.before(_yaw, _yaw_sp, _yawspeed_sp, dt);\n\tbool valid = _inputValid();\n\tgwm_trace::recorder.input_valid(valid);')
        text = replace_once(text, 'return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();',
            'return gwm_trace::recorder.result(valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite());')
    elif name == CMAKE:
        text = replace_once(text, '\t\tMulticopterPositionControl.cpp', '\t\tGwmMcTrace.cpp\n\t\tMulticopterPositionControl.cpp')
    else:
        raise ValueError(name)
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    if not args.build:
        print('No build. Explicit --build creates a distinct validation binary.'); return
    root = Path(os.environ.get('GWM_SIM_ROOT', str(Path.home()/'uav_autonomy'))).resolve()
    baseline = root/'upstream/PX4-Autopilot'
    pin = strict_json(SIM/'configs/versions.lock.yaml')['sources']['px4']['commit']
    if subprocess.check_output(['git','rev-parse','HEAD'], cwd=baseline, text=True).strip() != pin:
        raise ValueError('PX4 source pin mismatch')
    if subprocess.check_output(['git','status','--porcelain'], cwd=baseline, text=True).strip():
        raise ValueError('Pinned checkout is dirty')
    original_binary = baseline/'build/px4_sitl_default_linux/bin/px4'
    baseline_hash = digest(original_binary)
    if baseline_hash != strict_json(root/'state/p0-built.json')['measured']['px4_binary_sha256']:
        raise ValueError('Baseline binary receipt mismatch')
    files = tree_manifest(SIM/'validation/mc_trace')
    files['build_mc_trace.py'] = digest(__file__)
    identity = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    destination = root/'mc_trace_builds'/identity
    destination.mkdir(parents=True, exist_ok=False)
    source = destination/'PX4-Autopilot'
    started = time.time_ns()
    # A separate source and build tree retains native modes and submodule metadata.
    # Only the top-level generated build directory is excluded.
    shutil.copytree(baseline, source, symlinks=True,
        ignore=lambda directory,names: ['build'] if Path(directory)==baseline else [])
    changes = {}; diff = []
    for name in (CPP, CONTROL, CMAKE):
        before = (baseline/name).read_text(); after = instrument(before, name)
        (source/name).write_text(after)
        changes[name] = dict(before=digest(baseline/name), after=digest(source/name))
        diff.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True), 'a/'+name, 'b/'+name))
    (destination/'overlay.patch').write_text(''.join(diff))
    for name in ('GwmMcTrace.hpp','GwmMcTrace.cpp'):
        content = (SIM/'validation/mc_trace'/name).read_text()
        if name.endswith('.cpp'): content = '#define GWM_MC_OVERLAY_ID "'+identity+'"\n'+content
        (source/'src/modules/mc_pos_control'/name).write_text(content)
        changes['src/modules/mc_pos_control/'+name] = dict(before=None, after=digest(source/'src/modules/mc_pos_control'/name))
    receipt = dict(contract='px4-mc-causal-trace-v1', overlay_id=identity, source_commit=pin,
        overlay_files=files, source_changes=changes, source=str(source),
        baseline_binary=str(original_binary), baseline_binary_sha256=baseline_hash,
        patch_sha256=digest(destination/'overlay.patch'), started_unix_ns=started,
        status='building', clean_rebuild_claim=False)
    atomic_json(destination/'build-receipt.json', receipt)
    env = dict(os.environ, PATH=str(root/'venv-px4/bin')+':/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
               PYTHONNOUSERSITE='1')
    for key in ('PYTHONPATH','PYTHONHOME','CONDA_PREFIX','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH'):
        env.pop(key,None)
    # The baseline is built with the ROS Jazzy Gazebo vendor prefixes. Source the
    # same installed setup in a clean Linux environment; omitting it disables gz_bridge.
    resolved = subprocess.check_output(['bash','-c','source /opt/ros/jazzy/setup.bash; env -0'],env=env)
    env = dict(item.decode().split('=',1) for item in resolved.split(b'\0') if item)
    try:
        with (destination/'build.log').open('x') as log:
            subprocess.run(['make','-j2','px4_sitl','BUILD_DIR_SUFFIX=_mc_trace'], cwd=source, env=env,
                stdout=log, stderr=subprocess.STDOUT, check=True)
        build = source/'build/px4_sitl_default_mc_trace'
        binary = build/'bin/px4'
        if (build/'boardconfig').read_bytes() != (baseline/'build/px4_sitl_default_linux/boardconfig').read_bytes():
            raise ValueError('Instrumented board/module configuration differs from baseline')
        if not list(build.rglob('libmodules__simulation__gz_bridge.a')):
            raise ValueError('Gazebo bridge was not built')
        if (build/'parameters.json').read_bytes() != (baseline/'build/px4_sitl_default_linux/parameters.json').read_bytes():
            raise ValueError('Generated PX4 parameters differ from baseline')
        receipt.update(status='built', build=str(build), binary=str(binary), binary_sha256=digest(binary),
            elf_build_id=re.search(r'Build ID: (\w+)',subprocess.check_output(['readelf','-n',str(binary)],text=True)).group(1),
            completed_unix_ns=time.time_ns(), generated_headers={name:digest(build/'uORB/topics'/name)
                for name in ('vehicle_local_position.h','trajectory_setpoint.h','vehicle_local_position_setpoint.h')})
        if digest(original_binary)!=baseline_hash: raise ValueError('Baseline changed during isolated build')
        atomic_json(destination/'build-receipt.json', receipt)
        atomic_json(root/'state/px4-mc-trace-built.json',receipt)
        print(json.dumps(receipt,indent=2))
    except Exception as exc:
        receipt.update(status='failed', error=str(exc), completed_unix_ns=time.time_ns())
        atomic_json(destination/'build-receipt.json', receipt)
        raise


if __name__ == '__main__': main()
