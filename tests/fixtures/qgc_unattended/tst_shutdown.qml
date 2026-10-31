import QtQuick
import QtTest
import "."

TestCase {
    name: "PinnedQGCShutdown"
    when: windowShown
    property var window
    Component { id: factory; ShutdownWindow {} }
    function init() { QGroundControl.reset(); window = factory.createObject(null); verify(window !== null); wait(0) }
    function cleanup() { window.destroy(); wait(0) }
    function active() { QGroundControl.multiVehicleManager.activeVehicle = ({ id: 72 }) }
    function pending() { QGroundControl.multiVehicleManager.vehicles.entries = [{ parameterManager: { pendingWrites: true }}] }
    function dirty() { window.mission.dirtyForSave = true; window.mission.dirtyForUpload = true }
    function test_no_vehicle_data() { return [{tag:"interactive",mode:false},{tag:"explicit_unattended",mode:true}] }
    function test_no_vehicle(data) {
        window._gwmSitlActiveConnectionTeardown = data.mode
        verify(window.close()); compare(QGroundControl.calls.join(","), "link,video"); compare(window.dialogs.length,0)
    }
    function test_interactive_active_vehicle_still_requires_confirmation() {
        active(); verify(!window.close()); compare(window.dialogs[0],"Active Vehicle Connections")
        compare(QGroundControl.calls.length,0); verify(!window._forceClose)
    }
    function test_explicit_unattended_active_vehicle_uses_normal_finish() {
        active(); window._gwmSitlActiveConnectionTeardown=true
        verify(window.close()); compare(window.dialogs.length,0)
        compare(QGroundControl.calls.join(","),"link,video"); compare(window.promptClears,1)
    }
    function test_unsaved_mission_data() { return [{tag:"interactive",mode:false},{tag:"unattended",mode:true}] }
    function test_unsaved_mission(data) {
        active(); dirty(); window._gwmSitlActiveConnectionTeardown=data.mode
        verify(!window.close()); compare(window.dialogs[0],"Unsaved Mission")
        compare(QGroundControl.calls.length,0); verify(!window._forceClose); verify(window.mission.dirtyForSave)
    }
    function test_pending_writes_data() { return [{tag:"interactive",mode:false},{tag:"unattended",mode:true}] }
    function test_pending_writes(data) {
        active(); pending(); window._gwmSitlActiveConnectionTeardown=data.mode
        verify(!window.close()); compare(window.dialogs[0],"Pending Parameter Updates")
        compare(QGroundControl.calls.length,0); verify(!window._forceClose)
    }
    function test_explicit_user_mission_confirmation_still_checks_pending_writes() {
        active(); dirty(); pending(); window._gwmSitlActiveConnectionTeardown=true
        verify(!window.close()); window.acceptDialog()
        compare(window.dialogs.join(","),"Unsaved Mission,Pending Parameter Updates")
        compare(QGroundControl.calls.length,0)
    }
    function test_explicit_user_confirmation_then_normal_finish() {
        active(); dirty(); window._gwmSitlActiveConnectionTeardown=true
        verify(!window.close()); window.acceptDialog(); compare(QGroundControl.calls.join(","),"link,video")
    }
    function test_existing_uploaded_mission_recoverability_rule_unchanged() {
        active(); window.mission.dirtyForSave=true;window.mission.dirtyForUpload=false
        window._gwmSitlActiveConnectionTeardown=true;verify(window.close())
        compare(QGroundControl.calls.join(","),"link,video")
    }
    function test_reentrant_guard_still_blocks() {
        active();window._gwmSitlActiveConnectionTeardown=true;window._reentrantCloseGuard=true
        verify(!window.close());compare(QGroundControl.calls.length,0);verify(!window._forceClose)
    }
    function test_link_failure_cannot_claim_video_completion() {
        window._gwmSitlActiveConnectionTeardown=true;QGroundControl.failLink=true
        var failed=false;try { window.finishCloseProcess() } catch(e) { failed=true }
        verify(failed);compare(QGroundControl.calls.join(","),"link")
    }
}
