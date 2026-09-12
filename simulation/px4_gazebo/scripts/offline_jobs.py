"""Bounded P3 offline jobs; no flight or simulator entry point."""
import os
from pathlib import Path
import signal
import subprocess
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation'))
from p3_provenance import strict_json, require_evaluation, digest
from run_p2_repeated import check_smoke
from yaw_provenance import CONTRACT as YAW_CONTRACT


class OfflineJobError(ValueError):
    def __init__(self, stage, kind, reason):
        self.stage, self.kind = stage, kind
        super().__init__(stage+': '+reason)


def budget(sim):
    value = strict_json(Path(sim)/'configs/p3_offline_budget.json')
    if value.get('contract') != 'p3-offline-budget-v1': raise ValueError('Unsupported offline budget')
    for key in ('control_seconds','sensor_seconds','cleanup_seconds'):
        if type(value.get(key)) is not int or not 0 < value[key] <= 600:
            raise ValueError('Invalid finite offline budget: '+key)
    if value['cleanup_seconds'] != 10: raise ValueError('Unsupported cleanup budget')
    return value


def execute_offline(command, output, deadline):
    """Only the new process session is signalled, including surviving descendants."""
    if not 0 < deadline <= 600: raise ValueError('Invalid offline deadline')
    with Path(output).open('x') as stream:
        process = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            return process.wait(timeout=deadline)
        finally:
            # The leader may exit while its child still holds files. Always clean
            # the owned session group; never infer child exit from leader.poll().
            for sig in (signal.SIGTERM, getattr(signal,'SIGKILL',9)):
                try: os.killpg(process.pid,sig)
                except ProcessLookupError: pass
            if process.poll() is None: process.wait(timeout=10)


def require_current_result(run, name, frozen):
    value = require_evaluation(run,name,frozen)
    if (value.get('evaluation_finalized') is not True or value.get('analysis_inputs') != frozen
            or value.get('historical_reanalysis') is not False):
        raise ValueError('Unfinalized, stale or historical evaluator result')
    return value


def require_causal_control(control):
    evidence = control.get('yaw_ownership',{}).get('evidence',{})
    if (evidence.get('contract') != YAW_CONTRACT
            or any(evidence.get(key,{}).get('status') != 'verified' for key in
                ('A_external_initialization','B_handover','C_physical_behavior','D_exact_internal_relation'))):
        raise ValueError('Required yaw causal evidence is not verified')


def evaluate_trial(sim, run, batch, index, frozen, trial, entry, execute=None):
    """Return only after both finalized results pass; caller alone advances count."""
    execute = execute or execute_offline
    limits = budget(sim)
    entry['offline_budget'] = limits
    stage = 'control'
    try:
        names=('p2-offline-evaluation.json','p3-coexistence-evaluation.json')
        if any((run/name).exists() for name in names): raise ValueError('Preexisting evaluation output')
        code=execute(['bash',str(sim/'scripts/verify_p2_evidence.sh'),str(run)],
                     batch/f'control-{index}.log',limits['control_seconds'])
        entry['control_evaluator_exit']=code
        if code: raise ValueError('Control evaluator exited '+str(code))
        control=require_current_result(run,names[0],frozen)
        check_smoke(trial,control,frozen['files']['validation/collect_p2_evidence.py'])
        require_causal_control(control)
        entry['control_evaluation_sha256']=digest(run/names[0])
        stage='sensor'
        code=execute(['bash',str(sim/'scripts/verify_p3_coexistence.sh'),str(run)],
                     batch/f'sensor-{index}.log',limits['sensor_seconds'])
        entry['sensor_evaluator_exit']=code
        if code: raise ValueError('Sensor evaluator exited '+str(code))
        sensor=require_current_result(run,names[1],frozen)
        if sensor.get('status')!='passed': raise ValueError('Sensor composite did not pass')
        # Bind the dependent computation to these exact control-result bytes.
        if sensor.get('control_evaluation_sha256') != entry['control_evaluation_sha256']:
            raise ValueError('Sensor control dependency hash mismatch')
        if digest(run/names[0]) != entry['control_evaluation_sha256']:
            raise ValueError('Control result changed during sensor evaluation')
        entry['sensor_evaluation_sha256']=digest(run/names[1])
        return control,sensor
    except Exception as exc:
        kind='infrastructure_timeout' if isinstance(exc,subprocess.TimeoutExpired) else 'offline_evaluation_failure'
        entry.update(failure_kind=kind,failure_stage=stage,qualification_progress_valid=False)
        raise OfflineJobError(stage,kind,str(exc)) from exc
