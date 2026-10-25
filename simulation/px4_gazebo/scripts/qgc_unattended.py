"""Explicit verified SITL shutdown receipt and strict normal-teardown evidence gate.

Restored unchanged from the archived P3 work except: the native MAVLink decoder is
the consolidated p3_mavlink module, and wire evidence is its capture record.
"""
import json,os,re,time
from pathlib import Path
from p3_provenance import atomic_json,digest,strict_json

MODE='isolated-sitl-active-connection-v1'
PREFIX='GWM_SITL_SHUTDOWN_V1:'
REQUIRED=('TERM_ownership_verified','application_quit_event','normal_finish_begin',
 'qml_link_shutdown_begin','link_shutdown_enter','link_shutdown_return','qml_link_shutdown_return',
 'qml_video_stop_begin','qml_video_stop_return','normal_finish_reclose',
 'application_shutdown_enter','video_cleanup_enter','video_cleanup_return','application_shutdown_return')

def ownership_receipt(run,process,ground,clock,owner_script,expected_owner_sha256,*,now=time.monotonic_ns):
    if os.getpid()!=1:raise ValueError('QGC_shutdown_requires_owned_PID_namespace')
    if digest(owner_script)!=expected_owner_sha256:raise ValueError('QGC_guard_owner_source_changed')
    if not ground or ground['status'].get('arming_state')!=1 or ground['landed'].get('landed') is not True:
        raise ValueError('QGC_shutdown_ground_unconfirmed')
    stamp=now();age=stamp/1e9-ground['end_wall_s']
    if not 0<=age<=2. or process.poll() is not None:raise ValueError('QGC_shutdown_ground_or_process_stale')
    if clock is None or clock.get('error') or stamp/1e9-clock['progress_wall_s']>1.:
        raise ValueError('QGC_shutdown_clock_stale')
    fields=(Path('/proc/1/stat').read_text().rsplit(')',1)[1]).split()
    path=Path(run)/'qgc-shutdown-ownership.json'
    record=dict(contract='gwm-QGC-SITL-shutdown-ownership-v1',mode=MODE,owner_pid=1,
        owner_start_ticks=fields[19],qgc_pid=process.pid,uid=os.getuid(),health_verified=True,
        landed=True,arming_state=1,observed_monotonic_ns=stamp,owner_script=str(Path(owner_script).resolve()),
        owner_script_sha256=expected_owner_sha256,lock_path=str(Path('/proc/1/fd/9').resolve()))
    record.update({'namespace_'+name:os.readlink('/proc/self/ns/'+name) for name in ('pid','net','mnt','user')})
    atomic_json(path,record,exclusive=True);os.chmod(path,0o600)
    return record

def parse_stages(path):
    rows=[]
    for line in Path(path).read_text(errors='strict').splitlines():
        if PREFIX not in line:continue
        raw=line.split(PREFIX,1)[1]
        # Qt may append a source location after console.info; decode exactly one object.
        value,end=json.JSONDecoder().raw_decode(raw)
        if not isinstance(value,dict) or not isinstance(value.get('stage'),str):raise ValueError('QGC_shutdown_audit_malformed')
        rows.append(value)
    return rows

def require_teardown(log,cleanup):
    rows=parse_stages(log);stages=[row['stage'] for row in rows]
    if cleanup.get('exit_code')!=0 or cleanup.get('forced') or cleanup.get('registered_members_after'):
        raise ValueError('QGC_exit_or_resources_not_graceful')
    if 'TERM_ownership_rejected' in stages:raise ValueError('QGC_shutdown_ownership_rejected')
    cursor=-1
    for stage in REQUIRED:
        try:cursor=stages.index(stage,cursor+1)
        except ValueError as exc:raise ValueError('QGC_normal_cleanup_stage_missing_or_out_of_order:'+stage) from exc
    blocked=[row for row in rows if row.get('stage')=='close_check' and row.get('detail') in
        ('unsaved_mission_blocked','pending_parameter_writes_blocked','reentrant_guard_blocked')]
    if blocked:raise ValueError('QGC_protected_close_condition_encountered')
    totals=[row for row in rows if row['stage']=='native_attempt_audit_final']
    if len(totals)!=1 or totals[0].get('writer_healthy') is not True:raise ValueError('QGC_native_attempt_audit_not_finalized')
    for stage,key in [('command_attempt','command_attempts'),('outbound_attempt','outbound_attempts')]:
        ids={int(row['attempt_id']) for row in rows if row['stage']==stage}
        if ids!=set(range(1,int(totals[0][key])+1)):raise ValueError('QGC_native_attempt_audit_missing_sequence:'+stage)
        if stage=='command_attempt' and sum(row['stage']==stage for row in rows)!=int(totals[0][key]):
            raise ValueError('QGC_native_command_intent_duplicate_sequence')
    return dict(contract='gwm-QGC-normal-teardown-v1',status='passed',rows=rows,
        exit_code=0,normal_LinkManager_VideoManager_completed=True,grace_limit_s=5.)

def require_attempt_audit(log,wire):
    from collections import Counter
    from p3_mavlink import decode_datagram
    rows=parse_stages(log);groups={}
    for row in rows:
        if row['stage']=='command_attempt':
            values=row.get('parameters',[])
            if len(values)!=7 or any(v.get('native_bytes') not in (4,8) or len(bytes.fromhex(v['raw_hex']))!=v['native_bytes'] for v in values):
                raise ValueError('QGC_native_command_intent_parameters_incomplete')
        if row['stage']=='outbound_attempt':groups.setdefault(int(row['attempt_id']),[]).append(row)
    emitted=Counter()
    for chunks in groups.values():
        chunks.sort(key=lambda row:row['offset']);raw=bytearray();size=chunks[0]['total_bytes']
        for chunk in chunks:
            if chunk['offset']!=len(raw) or chunk['total_bytes']!=size:raise ValueError('QGC_outbound_audit_chunk_gap')
            raw.extend(bytes.fromhex(chunk['raw_hex']))
        if len(raw)!=size:raise ValueError('QGC_outbound_audit_truncated')
        emitted.update(row['raw_hex'] for row in decode_datagram(bytes(raw)) if 'command' in row)
    observed=Counter(row['raw_hex'] for row in wire['frames_of_interest'] if 'command' in row and row['source_system']!=72)
    if observed-emitted:raise ValueError('QGC_wire_command_missing_from_attempt_audit')
    return dict(status='passed',outbound_attempts=len(groups),command_intents=sum(row['stage']=='command_attempt' for row in rows),
        all_wire_commands_have_outbound_audit=True,raw_intents_preserved=True)
