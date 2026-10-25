"""PID-1 owner for one synthetic Qt signal/QML test; never starts QGC or a vehicle."""
from pathlib import Path
import fcntl,hashlib,json,os,signal,subprocess,sys,time

binary,window,output=map(Path,sys.argv[1:]);output.mkdir(exist_ok=False)
assert os.getpid()==1
run=output/'run';(run/'rootfs').mkdir(parents=True)
lock_path=output/'owner.lock';lock=os.open(lock_path,os.O_CREAT|os.O_RDWR,0o600)
if lock!=9:os.dup2(lock,9);os.close(lock)
fcntl.flock(9,fcntl.LOCK_EX|fcntl.LOCK_NB)
receipt=run/'qgc-shutdown-ownership.json';ready=output/'ready'
env=dict(os.environ,GWM_QGC_SHUTDOWN_MODE='isolated-sitl-active-connection-v1',GWM_QGC_SITL_SHUTDOWN_RECEIPT=str(receipt),
 GWM_QGC_AUDIT_NATIVE_ATTEMPTS='v1',QT_QPA_PLATFORM='offscreen',QT_QUICK_BACKEND='software')
with (output/'Qt-signal.log').open('x') as log:
 p=subprocess.Popen([str(binary),str(window),str(ready)],cwd=run/'rootfs',env=env,stdout=log,stderr=log,start_new_session=True)
 try:
  end=time.monotonic()+5.
  while not ready.exists() and p.poll() is None and time.monotonic()<end:time.sleep(.01)
  assert ready.exists(),'synthetic Qt readiness failed'
  fields=Path('/proc/1/stat').read_text().rsplit(')',1)[1].split();script=Path(__file__).resolve()
  row=dict(contract='gwm-QGC-SITL-shutdown-ownership-v1',mode='isolated-sitl-active-connection-v1',
   owner_pid=1,owner_start_ticks=fields[19],uid=os.getuid(),qgc_pid=p.pid,health_verified=True,landed=True,arming_state=1,
   observed_monotonic_ns=time.monotonic_ns(),owner_script=str(script),owner_script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),lock_path=str(lock_path))
  row.update({'namespace_'+name:os.readlink('/proc/self/ns/'+name) for name in ('pid','net','mnt','user')})
  receipt.write_text(json.dumps(row));receipt.chmod(0o600)
  begin=time.monotonic();p.send_signal(signal.SIGTERM);code=p.wait(timeout=5.)
  assert code==0,code
  elapsed=time.monotonic()-begin;assert elapsed<=5.
 finally:
  if p.poll() is None:p.kill();p.wait(timeout=2.)
text=(output/'Qt-signal.log').read_text()
for stage in ('TERM_ownership_verified','application_quit_event','active_connection_teardown_approved',
 'normal_finish_begin','qml_link_shutdown_return','qml_video_stop_return','normal_finish_reclose'):
 assert stage in text,stage
result=dict(status='passed',component='pinned SignalHandler + exact QGC Quit-event/QML bodies with manager test doubles',
 exit_code=code,TERM_to_exit_wall_s=elapsed,TERM_count=1,Qt_version='6.11.1',live_QGC=False,P3_credit=0)
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
