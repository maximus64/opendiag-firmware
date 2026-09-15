// SPDX-License-Identifier: GPL-3.0-only
#include "backend.h"
#include <stdexcept>

void Backend::load(const void *owner) {
    if (library_) {
        return;
    }
    std::string filename = platform::moduleDirectory(owner) + platform::nativeLibraryName();
    library_ = platform::openLibrary(filename);
    if (!library_) {
        throw std::runtime_error("Cannot load the adjacent opendiag32.dll backend");
    }
    try {
        scanForDevices =
            (PassThruScanForDevicesFn)platform::findSymbol(library_, "PassThruScanForDevices");
        if (!scanForDevices) {
            throw std::runtime_error("Missing backend export: PassThruScanForDevices");
        }
        getNextDevice =
            (PassThruGetNextDeviceFn)platform::findSymbol(library_, "PassThruGetNextDevice");
        if (!getNextDevice) {
            throw std::runtime_error("Missing backend export: PassThruGetNextDevice");
        }
        open = (PassThruOpenFn)platform::findSymbol(library_, "PassThruOpen");
        if (!open) {
            throw std::runtime_error("Missing backend export: PassThruOpen");
        }
        close = (PassThruCloseFn)platform::findSymbol(library_, "PassThruClose");
        if (!close) {
            throw std::runtime_error("Missing backend export: PassThruClose");
        }
        connect = (PassThruConnectFn)platform::findSymbol(library_, "PassThruConnect");
        if (!connect) {
            throw std::runtime_error("Missing backend export: PassThruConnect");
        }
        disconnect = (PassThruDisconnectFn)platform::findSymbol(library_, "PassThruDisconnect");
        if (!disconnect) {
            throw std::runtime_error("Missing backend export: PassThruDisconnect");
        }
        logicalConnect =
            (PassThruLogicalConnectFn)platform::findSymbol(library_, "OpenDiagLogicalConnect0404");
        if (!logicalConnect) {
            throw std::runtime_error("Missing backend export: OpenDiagLogicalConnect0404");
        }
        logicalDisconnect =
            (PassThruLogicalDisconnectFn)platform::findSymbol(library_,
                                                              "PassThruLogicalDisconnect");
        if (!logicalDisconnect) {
            throw std::runtime_error("Missing backend export: PassThruLogicalDisconnect");
        }
        select = (PassThruSelectFn)platform::findSymbol(library_, "PassThruSelect");
        if (!select) {
            throw std::runtime_error("Missing backend export: PassThruSelect");
        }
        readMsgs = (PassThruReadMsgsFn)platform::findSymbol(library_, "PassThruReadMsgs");
        if (!readMsgs) {
            throw std::runtime_error("Missing backend export: PassThruReadMsgs");
        }
        queueMsgs = (PassThruQueueMsgsFn)platform::findSymbol(library_, "PassThruQueueMsgs");
        if (!queueMsgs) {
            throw std::runtime_error("Missing backend export: PassThruQueueMsgs");
        }
        startPeriodicMsg =
            (PassThruStartPeriodicMsgFn)platform::findSymbol(library_, "PassThruStartPeriodicMsg");
        if (!startPeriodicMsg) {
            throw std::runtime_error("Missing backend export: PassThruStartPeriodicMsg");
        }
        stopPeriodicMsg =
            (PassThruStopPeriodicMsgFn)platform::findSymbol(library_, "PassThruStopPeriodicMsg");
        if (!stopPeriodicMsg) {
            throw std::runtime_error("Missing backend export: PassThruStopPeriodicMsg");
        }
        startMsgFilter =
            (PassThruStartMsgFilterFn)platform::findSymbol(library_, "PassThruStartMsgFilter");
        if (!startMsgFilter) {
            throw std::runtime_error("Missing backend export: PassThruStartMsgFilter");
        }
        stopMsgFilter =
            (PassThruStopMsgFilterFn)platform::findSymbol(library_, "PassThruStopMsgFilter");
        if (!stopMsgFilter) {
            throw std::runtime_error("Missing backend export: PassThruStopMsgFilter");
        }
        setProgrammingVoltage =
            (PassThruSetProgrammingVoltageFn)platform::findSymbol(library_,
                                                                  "PassThruSetProgrammingVoltage");
        if (!setProgrammingVoltage) {
            throw std::runtime_error("Missing backend export: PassThruSetProgrammingVoltage");
        }
        readVersion = (PassThruReadVersionFn)platform::findSymbol(library_, "PassThruReadVersion");
        if (!readVersion) {
            throw std::runtime_error("Missing backend export: PassThruReadVersion");
        }
        getLastError =
            (PassThruGetLastErrorFn)platform::findSymbol(library_, "PassThruGetLastError");
        if (!getLastError) {
            throw std::runtime_error("Missing backend export: PassThruGetLastError");
        }
        ioctl = (PassThruIoctlFn)platform::findSymbol(library_, "PassThruIoctl");
        if (!ioctl) {
            throw std::runtime_error("Missing backend export: PassThruIoctl");
        }
    } catch (...) {
        unload();
        throw;
    }
}

void Backend::unload() {
    if (library_) {
        platform::closeLibrary(library_);
        library_ = NULL;
    }
}
