pragma Singleton
import QtQml
QtObject {
    id: root
    property var calls: []
    property bool failLink: false
    property QtObject multiVehicleManager: QtObject {
        property var activeVehicle: null
        property QtObject vehicles: QtObject {
            property var entries: []
            readonly property int count: entries.length
            function get(index) { return entries[index] }
        }
    }
    property QtObject linkManager: QtObject {
        function shutdown() {
            root.calls = root.calls.concat(["link"])
            if (root.failLink) throw new Error("injected link teardown failure")
        }
    }
    property QtObject videoManager: QtObject {
        function stopVideo() { root.calls = root.calls.concat(["video"]) }
    }
    function reset() {
        calls = []; failLink = false
        multiVehicleManager.activeVehicle = null
        multiVehicleManager.vehicles.entries = []
    }
}
