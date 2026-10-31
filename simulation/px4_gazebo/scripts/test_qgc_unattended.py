"""Deterministic offline Qt/QML tests; never runs a QGC/PX4/Gazebo application."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM/'validation/qgc_unattended_v1'))
from overlay import block_end

def generate(source,destination):
    source=Path(source);destination=Path(destination);destination.mkdir(parents=True,exist_ok=False)
    fixture=SIM.parents[1]/'tests/fixtures/qgc_unattended'
    for name in ('qmldir','QGroundControl.qml','tst_shutdown.qml','guard_test.cpp'):
        shutil.copyfile(fixture/name,destination/name)
    raw=(source/'src/MainWindow/MainWindow.qml').read_text()
    begin=raw.index('    property bool _forceClose: false');end=block_end(raw,raw.index('    onClosing:',begin))+1
    body=raw[begin:end]
    header='''import QtQuick
import QtQuick.Window
import QtQuick.Controls
import "."
Window {
    id: mainWindow
    visible: true
    width: 100; height: 100
    property var dialogs: []
    property var acceptDialog: function() {}
    property int promptClears: 0
    property QtObject mission: QtObject { property bool dirtyForSave: false; property bool dirtyForUpload: false }
    property QtObject planView: QtObject { property var _planMasterController: mainWindow.mission }
    property QtObject firstRunPromptManager: QtObject { function clearNextPromptSignal() { mainWindow.promptClears++ } }
    function _showMessageDialogWorker(owner,title,text,buttons,accept,close,bypass) {
        dialogs = dialogs.concat([title]); acceptDialog = accept
    }
    Component.onCompleted: {
        if (typeof qtTestActive !== "undefined" && qtTestActive)
            QGroundControl.multiVehicleManager.activeVehicle = ({ id: 72 })
    }
'''
    (destination/'ShutdownWindow.qml').write_text(header+body+'\n}\n')
    identity=dict(upstream_file=str(source/'src/MainWindow/MainWindow.qml'),
        upstream_sha256=hashlib.sha256(raw.encode()).hexdigest(),exact_block_sha256=hashlib.sha256(body.encode()).hexdigest(),
        generated_fixture_sha256=hashlib.sha256((header+body+'\n}\n').encode()).hexdigest(),
        substitution='only dependencies are test doubles; close functions/onClosing copied byte-for-byte',live_QGC=False)
    (destination/'source-binding.json').write_text(json.dumps(identity,indent=2)+'\n')
    return identity

def signal_fixture(source,qt,output,env):
    """Use the real pinned POSIX handler and exact QGC event body in a Qt test app."""
    fixture=SIM.parents[1]/'tests/fixtures/qgc_unattended';base=output/'signal-source';base.mkdir()
    utilities=base/'Utilities';platform=utilities/'Platform';platform.mkdir(parents=True)
    for name in ('SignalHandler.cc','SignalHandler.h'):
        shutil.copyfile(source/'src/Utilities/Platform'/name,platform/name)
    shutil.copyfile(SIM/'validation/qgc_unattended_v1/GwmSitlShutdown.h',utilities/'GwmSitlShutdown.h')
    expected=hashlib.sha256((fixture/'owned_test_owner.py').read_bytes()).hexdigest()
    (utilities/'GwmSitlBuildIdentity.h').write_text('#pragma once\n#define GWM_QGC_OWNER_SOURCE_SHA256 "'+expected+'"\n')
    (platform/'QGCLoggingCategory.h').write_text('#include <QtCore/QLoggingCategory>\n#define QGC_LOGGING_CATEGORY(name, text) Q_LOGGING_CATEGORY(name, text)\n')
    app=(source/'src/QGCApplication.cc').read_text();start=app.index('bool QGCApplication::event(QEvent* e)')
    event=app[start:block_end(app,start)+1].replace('QGCApplication::event','App::event',1)
    prefix='''#include <QtGui/QGuiApplication>
#include <QtGui/QWindow>
#include <QtQml/QQmlApplicationEngine>
#include <QtQml/QQmlContext>
#include <QtCore/QLoggingCategory>
#include <QtCore/QFile>
#include "Utilities/GwmSitlShutdown.h"
#include "Utilities/Platform/SignalHandler.h"
Q_LOGGING_CATEGORY(QGCApplicationLog,"QtFixture")
class App : public QGuiApplication { public: using QGuiApplication::QGuiApplication; QWindow* _mainRootWindow=nullptr; bool event(QEvent*) override; };
'''
    tail='''
int main(int argc,char** argv) {
    App app(argc,argv); QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("qtTestActive",true);
    engine.load(QUrl::fromLocalFile(QString::fromLocal8Bit(argv[1])));
    if(engine.rootObjects().isEmpty()) return 2;
    app._mainRootWindow=qobject_cast<QWindow*>(engine.rootObjects()[0]);
    SignalHandler handler; if(handler.setupSignalHandlers()!=0) return 3;
    QFile ready(QString::fromLocal8Bit(argv[2])); if(!ready.open(QIODevice::WriteOnly)) return 4; ready.write("ready");ready.close();
    return app.exec();
}
'''
    (base/'main.cpp').write_text(prefix+event+tail)
    steps=[([str(qt/'libexec/moc'),str(platform/'SignalHandler.h'),'-o',str(base/'moc.cpp')],'signal-moc.log')]
    includes=['-I'+str(base),'-I'+str(platform),'-I'+str(qt/'include')]+['-I'+str(qt/'include'/name) for name in ('QtCore','QtGui','QtQml')]
    binary=base/'qt-signal-fixture'
    steps.append((['g++','-std=c++17','-fPIC','-DGWM_QGC_SITL_SHUTDOWN_BUILD=1',*includes,
        str(base/'main.cpp'),str(platform/'SignalHandler.cc'),str(base/'moc.cpp'),'-L'+str(qt/'lib'),'-Wl,-rpath,'+str(qt/'lib'),
        '-lQt6Qml','-lQt6Gui','-lQt6Core','-o',str(binary)],'signal-compile.log'))
    # Ownership receipts require native Linux mode 0600, as the real owner does.
    # A Windows/DrvFS test-output path cannot satisfy that protection.
    native_result=source.parent/'offline-signal-tests'/output.name
    native_result.parent.mkdir(exist_ok=True)
    steps.append((['unshare','--user','--map-current-user','--keep-caps','--net','--mount','--pid','--fork','--mount-proc',
        '/usr/bin/python3',str(fixture/'owned_test_owner.py'),str(binary),str(output/'qml/ShutdownWindow.qml'),str(native_result)],'signal-run.log'))
    return steps,dict(signal_handler_sha256=hashlib.sha256((platform/'SignalHandler.cc').read_bytes()).hexdigest(),
        exact_QGC_event_body_sha256=hashlib.sha256(event.encode()).hexdigest(),fixture_owner_sha256=expected,
        native_result=str(native_result),production_owner_replaced_only_in_component_test=True)

def run(source,qt,output):
    source=Path(source);qt=Path(qt);output=Path(output);identity=generate(source,output/'qml')
    env=dict(os.environ,QT_QPA_PLATFORM='offscreen',QT_QUICK_BACKEND='software',
        LD_LIBRARY_PATH=str(qt/'lib'),QT_PLUGIN_PATH=str(qt/'plugins'),QML_IMPORT_PATH=str(qt/'qml'))
    commands=[([str(qt/'bin/qmltestrunner'),'-input',str(output/'qml'),'-o',str(output/'qml-results.xml')+',junitxml'], 'qml.log')]
    include=SIM/'validation/qgc_unattended_v1'
    (output/'GwmSitlBuildIdentity.h').write_text('#pragma once\n#define GWM_QGC_OWNER_SOURCE_SHA256 "offline-test-not-production-owner"\n')
    shutil.copyfile(include/'GwmSitlShutdown.h',output/'GwmSitlShutdown.h')
    for flag,label in [([], 'interactive-default'),(['-DGWM_QGC_SITL_SHUTDOWN_BUILD=1'],'compiled-opt-in')]:
        binary=output/label
        compile_cmd=['g++','-std=c++17','-fPIC',*flag,'-I'+str(output),'-I'+str(qt/'include'),'-I'+str(qt/'include/QtCore'),
            str(output/'qml/guard_test.cpp'),'-L'+str(qt/'lib'),'-Wl,-rpath,'+str(qt/'lib'),'-lQt6Core','-o',str(binary)]
        commands.extend([(compile_cmd,label+'-compile.log'),([str(binary)],label+'.log')])
    steps,signal_identity=signal_fixture(source,qt,output,env);commands.extend(steps)
    results=[]
    for command,name in commands:
        with (output/name).open('x') as stream:
            process=subprocess.run(command,env=env,stdout=stream,stderr=stream,timeout=45)
        results.append(dict(command=command,log=name,exit_code=process.returncode))
        if process.returncode:break
    receipt=dict(contract='gwm-QGC-offline-Qt-tests-v1',source_binding=identity,Qt_sdk=str(qt),
        signal_fixture=signal_identity,
        results=results,status='passed' if len(results)==len(commands) and not any(r['exit_code'] for r in results) else 'failed',
        live_graceful_shutdown_proven=False,runtime_campaigns=0)
    native_result=Path(signal_identity['native_result'])
    if native_result.exists():shutil.copytree(native_result,output/'signal-result')
    (output/'result.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(dict(status=receipt['status'],output=str(output),steps=len(results))))
    return 0 if receipt['status']=='passed' else 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True,type=Path);p.add_argument('--qt',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();raise SystemExit(run(a.source,a.qt,a.output))
