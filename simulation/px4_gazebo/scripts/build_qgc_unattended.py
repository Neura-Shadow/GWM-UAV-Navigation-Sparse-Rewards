"""Offline-only versioned QGC source overlay and bundle freeze; never executes QGC."""
import argparse,hashlib,json,os,re,shutil,subprocess,sys
from pathlib import Path
SIM=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(SIM/'validation'),str(SIM/'validation/qgc_unattended_v1')]
from p3_provenance import atomic_json,digest,strict_json
from overlay import apply,COMMIT

def freeze(source,elf,stock,destination,overlay_receipt,qt_test_receipt,output):
    source,elf,stock,destination=map(Path,(source,elf,stock,destination))
    overlay=strict_json(overlay_receipt);tests=strict_json(qt_test_receipt)
    owner_sha=digest(SIM/'scripts/p3_namespace_owner.py')
    if (overlay['source_commit']!=COMMIT or overlay['owner_script_sha256']!=owner_sha
            or tests['status']!='passed' or tests['live_graceful_shutdown_proven'] is not False):
        raise ValueError('QGC_source_owner_or_offline_test_receipt_unconfirmed')
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()!=COMMIT:raise ValueError('QGC_source_revision_changed')
    for name,expected in overlay['after'].items():
        if digest(source/name)!=expected:raise ValueError('QGC_overlay_source_changed:'+name)
    if digest(source/'src/Utilities/GwmSitlShutdown.h')!=digest(SIM/'validation/qgc_unattended_v1/GwmSitlShutdown.h'):
        raise ValueError('QGC_guard_differs_from_reviewed_source')
    destination.mkdir(parents=True,exist_ok=False);app=destination/'squashfs-root';app.mkdir()
    original=stock/'squashfs-root'
    for path in original.iterdir():
        if path.name=='usr':continue
        if path.name=='AppRun':shutil.copy2(path,app/path.name)
        else:(app/path.name).symlink_to(path,target_is_directory=path.is_dir())
    (app/'usr').mkdir()
    for path in (original/'usr').iterdir():
        if path.name in ('bin','lib'):continue
        (app/'usr'/path.name).symlink_to(path,target_is_directory=path.is_dir())
    for folder in ('bin','lib'):
        target=app/'usr'/folder;target.mkdir()
        for path in (original/'usr'/folder).iterdir():
            if folder=='bin' and path.name=='QGroundControl':continue
            (target/path.name).symlink_to(path,target_is_directory=path.is_dir())
    shutil.copy2(elf,app/'usr/bin/QGroundControl');(app/'usr/bin/QGroundControl').chmod(0o755)
    # New source-built loadable plugins/libraries override their stock counterpart.
    built_libraries={}
    for path in sorted(elf.parent.rglob('*.so*')):
        if not path.is_file():continue
        relative=path.relative_to(elf.parent);parts=relative.parts
        suffix=Path(*parts[1:]) if parts and parts[0]=='lib' else relative
        target=app/'usr/lib'/suffix
        if target.parent.is_symlink():
            prior=target.parent.resolve();target.parent.unlink();target.parent.mkdir()
            for existing in prior.iterdir():(target.parent/existing.name).symlink_to(existing,target_is_directory=existing.is_dir())
        target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists() or target.is_symlink():target.unlink()
        shutil.copy2(path,target);built_libraries[str(target)]=digest(target)
    (destination/'QGroundControl-x86_64.AppImage').symlink_to(stock/'QGroundControl-x86_64.AppImage')
    env=dict(os.environ,LD_LIBRARY_PATH=str(app/'usr/lib'))
    dependencies=subprocess.run(['ldd','-r',str(app/'usr/bin/QGroundControl')],env=env,text=True,capture_output=True,timeout=60)
    (destination/'relocations.txt').write_text(dependencies.stdout+dependencies.stderr)
    if dependencies.returncode or any(word in dependencies.stdout+dependencies.stderr for word in ('not found','undefined symbol')):
        raise ValueError('custom_QGC_library_relocations_unresolved')
    native={};cache={}
    for directory,dirs,files in os.walk(destination,followlinks=True):
        for name in files:
            path=Path(directory)/name
            if not path.is_file():continue
            resolved=path.resolve()
            if resolved not in cache:cache[resolved]=digest(path)
            native[str(path)]=cache[resolved]
    # ELF's selected system dependencies and loader are part of the identity too.
    resolved_libraries={}
    for line in dependencies.stdout.splitlines():
        matches=re.findall(r'(?:=>\s+|^\s*)(/[^\s]+)',line)
        for name in matches:
            path=Path(name)
            if path.is_file():resolved_libraries[str(path)]=digest(path)
    native.update(resolved_libraries)
    result=dict(contract='gwm-QGC-unattended-build-v1',status='built_offline_not_runtime_verified',source_commit=COMMIT,
        owner_script_sha256=owner_sha,overlay_receipt=dict(path=str(Path(overlay_receipt)),sha256=digest(overlay_receipt)),
        offline_Qt_tests=dict(path=str(Path(qt_test_receipt)),sha256=digest(qt_test_receipt)),
        bundle=str(destination),ELF=str(app/'usr/bin/QGroundControl'),ELF_sha256=digest(app/'usr/bin/QGroundControl'),
        base_stock_ELF_sha256=digest(original/'usr/bin/QGroundControl'),native_files=native,source_built_libraries=built_libraries,
        resolved_dynamic_libraries=resolved_libraries,
        command_generation='stock unchanged; A/B decision unapproved',shutdown_mode='isolated-sitl-active-connection-v1',
        original_QGC_grace_s=5,live_graceful_shutdown_proven=False,acceptance_timing_receipts=[])
    atomic_json(output,result,exclusive=True);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--prepare',action='store_true')
    p.add_argument('--elf',type=Path);p.add_argument('--stock',type=Path);p.add_argument('--destination',type=Path)
    p.add_argument('--overlay-receipt',type=Path);p.add_argument('--qt-tests',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.prepare:atomic_json(a.output,apply(a.source,digest(SIM/'scripts/p3_namespace_owner.py')),exclusive=True)
    else:
        if not all((a.elf,a.stock,a.destination,a.overlay_receipt,a.qt_tests)):p.error('freeze requires all exact build/test/bundle inputs')
        result=freeze(a.source,a.elf,a.stock,a.destination,a.overlay_receipt,a.qt_tests,a.output)
        print(json.dumps(dict(ELF_sha256=result['ELF_sha256'],native_files=len(result['native_files']),runtime='not_run')))
