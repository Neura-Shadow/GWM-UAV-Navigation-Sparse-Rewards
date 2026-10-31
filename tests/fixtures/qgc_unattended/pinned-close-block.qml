    property bool _forceClose: false
    property bool suppressCriticalVehicleMessages: false

    function finishCloseProcess() {
        _forceClose = true
        // For some reason on the Qml side Qt doesn't automatically disconnect a signal when an object is destroyed.
        // So we have to do it ourselves otherwise the signal flows through on app shutdown to an object which no longer exists.
        firstRunPromptManager.clearNextPromptSignal()
        QGroundControl.linkManager.shutdown()
        QGroundControl.videoManager.stopVideo();
        mainWindow.close()
    }

    // Check for things which should prevent the app from closing
    //  Returns true if it is OK to close
    readonly property int _skipUnsavedMissionCheckMask: 0x01
    readonly property int _skipPendingParameterWritesCheckMask: 0x02
    readonly property int _skipActiveConnectionsCheckMask: 0x04
    property int _closeChecksToSkip: 0
    property bool _reentrantCloseGuard: false
    function performCloseChecks() {
        if (!(_closeChecksToSkip & _skipUnsavedMissionCheckMask) && !checkForUnsavedMission()) {
            return false
        }
        if (!(_closeChecksToSkip & _skipPendingParameterWritesCheckMask) && !checkForPendingParameterWrites()) {
            return false
        }
        if (!(_closeChecksToSkip & _skipActiveConnectionsCheckMask) && !checkForActiveConnections()) {
            return false
        }
        finishCloseProcess()
        return true
    }

    function checkForUnsavedMission() {
        // Only warn when edits are neither saved to disk nor uploaded to the vehicle.
        // If either happened the edits are recoverable, so closing loses nothing.
        // With no active vehicle an upload can't have happened, so treat the plan as
        // not uploaded regardless of dirtyForUpload.
        if (planView._planMasterController.dirtyForSave &&
                (planView._planMasterController.dirtyForUpload || !QGroundControl.multiVehicleManager.activeVehicle)) {
            let accepted = false
            _reentrantCloseGuard = true
            _showMessageDialogWorker(mainWindow, qsTr("Unsaved Mission"),
                              qsTr("You have a mission edit in progress which has not been saved/uploaded. If you close you will lose changes. Are you sure you want to close?"),
                              Dialog.Yes | Dialog.No,
                              function() { accepted = true; _closeChecksToSkip |= _skipUnsavedMissionCheckMask; performCloseChecks() },
                              function() { if (!accepted) _reentrantCloseGuard = false },
                              true /* bypassNavigationCheck */)
            return false
        } else {
            return true
        }
    }

    function checkForPendingParameterWrites() {
        for (var index=0; index<QGroundControl.multiVehicleManager.vehicles.count; index++) {
            if (QGroundControl.multiVehicleManager.vehicles.get(index).parameterManager.pendingWrites) {
                let accepted = false
                _reentrantCloseGuard = true
                _showMessageDialogWorker(mainWindow, qsTr("Pending Parameter Updates"),
                    qsTr("You have pending parameter updates to a vehicle. If you close you will lose changes. Are you sure you want to close?"),
                    Dialog.Yes | Dialog.No,
                    function() { accepted = true; _closeChecksToSkip |= _skipPendingParameterWritesCheckMask; performCloseChecks() },
                    function() { if (!accepted) _reentrantCloseGuard = false },
                    true /* bypassNavigationCheck */)
                return false
            }
        }
        return true
    }

    function checkForActiveConnections() {
        if (QGroundControl.multiVehicleManager.activeVehicle) {
            let accepted = false
            _reentrantCloseGuard = true
            _showMessageDialogWorker(mainWindow, qsTr("Active Vehicle Connections"),
                qsTr("There are still active connections to vehicles. Are you sure you want to exit?"),
                Dialog.Yes | Dialog.No,
                function() { accepted = true; _closeChecksToSkip |= _skipActiveConnectionsCheckMask; performCloseChecks() },
                function() { if (!accepted) _reentrantCloseGuard = false },
                true /* bypassNavigationCheck */)
            return false
        } else {
            return true
        }
    }

    onClosing: (close) => {
        if (!_forceClose) {
            if (_reentrantCloseGuard) {
                close.accepted = false
                return
            }
            _closeChecksToSkip = 0
            close.accepted = performCloseChecks()
        }
    }
