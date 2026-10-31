"""Ordinary verification receipt; no simulator, flight, build or permission changes."""
import argparse
import compileall
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM/'validation'))
from p3_provenance import atomic_json,digest,frozen_inputs
from mc_trace_runtime import selected_build


def main():
    p=argparse.ArgumentParser(); p.add_argument('--verify',action='store_true'); args=p.parse_args()
    if not args.verify: print('No verification. Explicit --verify required.'); return
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    local=SIM.parents[1]/'results/px4_mc_trace'
    frozen=frozen_inputs(SIM)
    tests=ET.parse(local/'full-tests.xml').getroot()
    failures=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in tests.findall('testsuite'))
    count=sum(int(s.get('tests',0)) for s in tests.findall('testsuite'))
    skipped=sum(int(s.get('skipped',0)) for s in tests.findall('testsuite'))
    if failures or count<1292: raise ValueError('Complete ordinary test suite required')
    if not compileall.compile_dir(str(SIM/'scripts'),quiet=1) or not compileall.compile_dir(str(SIM/'validation'),quiet=1):
        raise ValueError('Python compilation failed')
    scripts=sorted((SIM/'scripts').glob('*.sh'))
    for script in scripts: subprocess.run(['bash','-n',str(script)],check=True)
    subprocess.run(['shellcheck','-x','-e','SC1091',*[str(p) for p in scripts]],check=True)
    # This is the shared Windows checkout. Match its CRLF and NTFS mode
    # interpretation for this read-only invocation; do not rewrite Git config.
    diff=subprocess.run(['git','-c','core.autocrlf=true','-c','core.filemode=false','diff','--check'],
        cwd=SIM,check=False,capture_output=True,text=True)
    (local/'linux-diff-check.log').write_text(diff.stdout+diff.stderr)
    if diff.returncode: raise ValueError('Diff check failed; see linux-diff-check.log')
    build=selected_build(root,SIM)
    if not build: raise ValueError('Explicit instrumented build required')
    if frozen_inputs(SIM)!=frozen: raise ValueError('Verification inputs changed')
    report=dict(status='passed',frozen_inputs=frozen,build=build,tests=count,skipped=skipped,failures=failures,
        tests_xml_sha256=digest(local/'full-tests.xml'),tests_log_sha256=digest(local/'full-tests.log'),
        python_compile=True,bash_syntax=True,shellcheck=True,diff_check=True,
        package_source_changes=False,installed_package_rebuild_claim=False,
        completed_unix_ns=time.time_ns())
    path=root/'state'/('mc-trace-verification-'+str(report['completed_unix_ns'])+'.json')
    atomic_json(path,report,exclusive=True)
    print('MC ordinary verification: '+str(path),flush=True)


if __name__=='__main__': main()
