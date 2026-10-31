"""Pinned-source additive shutdown overlay. No build, process or policy activation on import."""
from pathlib import Path
import hashlib,json,re,subprocess

COMMIT='4568b585648e8c6065cca6c650b52731c1d3baa3'
MODE='isolated-sitl-active-connection-v1'
ROOT=Path(__file__).resolve().parent

def once(text,old,new):
    if text.count(old)!=1:raise ValueError('pinned_source_anchor_missing_or_ambiguous:'+old[:70])
    return text.replace(old,new,1)

def block_end(text,start):
    """Balanced C++/QML block, excluding quoted strings and comments."""
    begin=text.index('{',start);depth=0;quote=None;i=begin
    while i<len(text):
        c=text[i]
        if quote:
            if c=='\\':i+=2;continue
            if c==quote:quote=None
        elif text.startswith('//',i):i=text.index('\n',i);continue
        elif text.startswith('/*',i):i=text.index('*/',i+2)+2;continue
        elif c in ('"',"'",'`'):quote=c
        elif c=='{':depth+=1
        elif c=='}':
            depth-=1
            if not depth:return i
        i+=1
    raise ValueError('source_block_incomplete')

def instrument_function(text,signature,enter,leave):
    start=text.index(signature);begin=text.index('{',start);end=block_end(text,start)
    return text[:begin+1]+'\n    GwmSitlShutdown::stage("'+enter+'");'+text[begin+1:end]+'    GwmSitlShutdown::stage("'+leave+'");\n'+text[end:]

def qml_overlay(text):
    text=once(text,'    property bool suppressCriticalVehicleMessages: false', '''    property bool suppressCriticalVehicleMessages: false
    // Set only by a compiled SITL integration after an independently verified TERM receipt.
    property bool _gwmSitlActiveConnectionTeardown: false
    function gwmShutdownStage(stage, detail) {
        if (_gwmSitlActiveConnectionTeardown)
            console.info("GWM_SITL_SHUTDOWN_V1:" + JSON.stringify({stage: stage, detail: detail}))
    }''')
    text=once(text,'    function finishCloseProcess() {','    function finishCloseProcess() {\n        gwmShutdownStage("normal_finish_begin", "")')
    text=once(text,'        QGroundControl.linkManager.shutdown()','        gwmShutdownStage("qml_link_shutdown_begin", "")\n        QGroundControl.linkManager.shutdown()\n        gwmShutdownStage("qml_link_shutdown_return", "")')
    text=once(text,'        QGroundControl.videoManager.stopVideo();','        gwmShutdownStage("qml_video_stop_begin", "")\n        QGroundControl.videoManager.stopVideo();\n        gwmShutdownStage("qml_video_stop_return", "")')
    text=once(text,'        mainWindow.close()','        gwmShutdownStage("normal_finish_reclose", "")\n        mainWindow.close()')
    text=once(text,'            let accepted = false\n            _reentrantCloseGuard = true\n            _showMessageDialogWorker(mainWindow, qsTr("Unsaved Mission"),',
        '            gwmShutdownStage("close_check", "unsaved_mission_blocked")\n            let accepted = false\n            _reentrantCloseGuard = true\n            _showMessageDialogWorker(mainWindow, qsTr("Unsaved Mission"),')
    text=once(text,'            if (QGroundControl.multiVehicleManager.vehicles.get(index).parameterManager.pendingWrites) {',
        '            if (QGroundControl.multiVehicleManager.vehicles.get(index).parameterManager.pendingWrites) {\n                gwmShutdownStage("close_check", "pending_parameter_writes_blocked")')
    text=once(text,'        if (QGroundControl.multiVehicleManager.activeVehicle) {', '''        if (QGroundControl.multiVehicleManager.activeVehicle) {
            gwmShutdownStage("close_check", "active_vehicle_connection")
            if (_gwmSitlActiveConnectionTeardown) {
                gwmShutdownStage("active_connection_teardown_approved", "normal_finish_path")
                return true
            }''')
    text=once(text,'            if (_reentrantCloseGuard) {','            if (_reentrantCloseGuard) {\n                gwmShutdownStage("close_check", "reentrant_guard_blocked")')
    return text

def apply(source,owner_sha256):
    source=Path(source)
    actual=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if actual!=COMMIT:raise ValueError('QGC_source_revision_changed')
    if subprocess.check_output(['git','status','--porcelain'],cwd=source):raise ValueError('overlay_requires_clean_pinned_source')
    before={};after={}
    def edit(path,action):
        p=source/path;raw=p.read_bytes();before[path]=hashlib.sha256(raw).hexdigest()
        result=action(raw.decode());p.write_text(result);after[path]=hashlib.sha256(p.read_bytes()).hexdigest()
    edit('src/MainWindow/MainWindow.qml',qml_overlay)
    edit('src/Utilities/Platform/SignalHandler.cc',lambda text:once('#include "../GwmSitlShutdown.h"\n'+text,
        '    qCDebug(SignalHandlerLog) << "Caught SIGTERM—shutting down gracefully";',
        '    GwmSitlShutdown::onTerm();\n    qCDebug(SignalHandlerLog) << "Caught SIGTERM—shutting down gracefully";'))
    def application(text):
        text='#include "Utilities/GwmSitlShutdown.h"\n'+text
        text=once(text,'            _mainRootWindow->close();', '''            if (GwmSitlShutdown::armed()) {
                GwmSitlShutdown::stage("application_quit_event");
                _mainRootWindow->setProperty("_gwmSitlActiveConnectionTeardown", true);
            }
            const bool gwmCloseAccepted = _mainRootWindow->close();
            if (GwmSitlShutdown::armed())
                GwmSitlShutdown::audit("window_close_return", {{"accepted", gwmCloseAccepted}});''')
        text=instrument_function(text,'void QGCApplication::shutdown()', 'application_shutdown_enter','application_shutdown_return')
        return once(text,'    GwmSitlShutdown::stage("application_shutdown_return");',
            '    GwmSitlShutdown::stage("application_shutdown_return");\n    GwmSitlShutdown::auditTotals();')
    edit('src/QGCApplication.cc',application)
    edit('src/Comms/LinkManager.cc',lambda text:instrument_function('#include "../Utilities/GwmSitlShutdown.h"\n'+text,
        'void LinkManager::shutdown()', 'link_shutdown_enter','link_shutdown_return'))
    video='src/VideoManager/VideoManager.cc'
    edit(video,lambda text:instrument_function('#include "../Utilities/GwmSitlShutdown.h"\n'+text,
        'void VideoManager::cleanup()', 'video_cleanup_enter','video_cleanup_return'))
    edit('src/Comms/LinkInterface.cc',lambda text:once('#include "../Utilities/GwmSitlShutdown.h"\n'+text,
        '    const QByteArray data(bytes, length);','    const QByteArray data(bytes, length);\n    GwmSitlShutdown::outboundAttempt(data);'))
    def vehicle(text):
        text='#include "../Utilities/GwmSitlShutdown.h"\n'+text
        signatures=re.findall(r'void Vehicle::(sendMavCommand(?:Delayed|WithHandler|Int|IntWithHandler|WithLambdaFallback|IntWithLambdaFallback)?)\([^\n]*\)\n\{',text)
        if len(signatures)!=7:raise ValueError('command_API_audit_coverage_changed:'+str(signatures))
        for name in signatures:
            start=text.index('void Vehicle::'+name+'(');brace=text.index('{',start)
            text=text[:brace+1]+'\n    GwmSitlShutdown::commandAttempt(int(command), compId, param1, param2, param3, param4, param5, param6, param7);'+text[brace+1:]
        return text
    edit('src/Vehicle/Vehicle.cc',vehicle)
    edit('CMakeLists.txt',lambda text:once(text,'find_package(Python3 REQUIRED COMPONENTS Interpreter)', '''option(GWM_QGC_SITL_SHUTDOWN_BUILD "Versioned isolated SITL shutdown integration" OFF)
if(GWM_QGC_SITL_SHUTDOWN_BUILD)
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        message(FATAL_ERROR "GWM unattended integration requires isolated Linux SITL")
    endif()
    add_compile_definitions(GWM_QGC_SITL_SHUTDOWN_BUILD=1)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)'''))
    for name,raw in [('GwmSitlShutdown.h',(ROOT/'GwmSitlShutdown.h').read_bytes()),
        ('GwmSitlBuildIdentity.h',('#pragma once\n#define GWM_QGC_OWNER_SOURCE_SHA256 "'+owner_sha256+'"\n').encode())]:
        p=source/'src/Utilities'/name;p.write_bytes(raw);after[str(p.relative_to(source))]=hashlib.sha256(raw).hexdigest()
    return dict(contract='gwm-QGC-unattended-shutdown-overlay-v1',source_commit=COMMIT,mode=MODE,
        owner_script_sha256=owner_sha256,before=before,after=after,command_policy='stock_requests_unchanged_no_waiver',
        interactive_default='unchanged',runtime_proof='not_run',shutdown_grace_s=5)
