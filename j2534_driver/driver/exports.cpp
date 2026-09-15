// SPDX-License-Identifier: GPL-3.0-only
#include "../common/api_trace.h"
#include "../include/j2534.h"
#include <exception>

extern const void *moduleAnchor;

namespace host {
bool enterApi();
void leaveApi();
J2534_LONG finishApi(J2534_LONG status, const char *message = NULL);
J2534_LONG PassThruScanForDevices(J2534_ULONG *pDeviceCount);
J2534_LONG PassThruGetNextDevice(SDEVICE *psDevice);
J2534_LONG PassThruOpen(const char *pName, J2534_ULONG *pDeviceID);
J2534_LONG PassThruClose(J2534_ULONG DeviceID);
J2534_LONG PassThruConnect(J2534_ULONG DeviceID,
                           J2534_ULONG ProtocolID,
                           J2534_ULONG Flags,
                           J2534_ULONG BaudRate,
                           RESOURCE_STRUCT ResourceStruct,
                           J2534_ULONG *pChannelID);
J2534_LONG PassThruDisconnect(J2534_ULONG ChannelID);
J2534_LONG PassThruLogicalConnect(J2534_ULONG PhysicalChannelID,
                                  J2534_ULONG ProtocolID,
                                  J2534_ULONG Flags,
                                  void *pChannelDescriptor,
                                  J2534_ULONG *pChannelID,
                                  bool legacyChannel);
J2534_LONG PassThruLogicalDisconnect(J2534_ULONG ChannelID);
J2534_LONG PassThruSelect(SCHANNELSET *ChannelSetPtr, J2534_ULONG SelectType, J2534_ULONG Timeout);
J2534_LONG PassThruReadMsgs(J2534_ULONG ChannelID,
                            PASSTHRU_MSG *pMsg,
                            J2534_ULONG *pNumMsgs,
                            J2534_ULONG Timeout);
J2534_LONG PassThruQueueMsgs(J2534_ULONG ChannelID, PASSTHRU_MSG *pMsg, J2534_ULONG *pNumMsgs);
J2534_LONG PassThruStartPeriodicMsg(J2534_ULONG ChannelID,
                                    PASSTHRU_MSG *pMsg,
                                    J2534_ULONG *pMsgID,
                                    J2534_ULONG TimeInterval);
J2534_LONG PassThruStopPeriodicMsg(J2534_ULONG ChannelID, J2534_ULONG MsgID);
J2534_LONG PassThruStartMsgFilter(J2534_ULONG ChannelID,
                                  J2534_ULONG FilterType,
                                  PASSTHRU_MSG *pMaskMsg,
                                  PASSTHRU_MSG *pPatternMsg,
                                  J2534_ULONG *pFilterID);
J2534_LONG PassThruStopMsgFilter(J2534_ULONG ChannelID, J2534_ULONG FilterID);
J2534_LONG PassThruSetProgrammingVoltage(J2534_ULONG DeviceID,
                                         RESOURCE_STRUCT ResourceStruct,
                                         J2534_ULONG Voltage);
J2534_LONG PassThruReadVersion(J2534_ULONG DeviceID,
                               char *pFirmwareVersion,
                               char *pDllVersion,
                               char *pApiVersion);
J2534_LONG PassThruGetLastError(char *pErrorDescription);
J2534_LONG PassThruIoctl(J2534_ULONG ControlTarget,
                         J2534_ULONG IoctlID,
                         void *InputPtr,
                         void *OutputPtr);
} // namespace host

// Serialize access to DLL state without holding a lock across a second caller.
class ApiGuard {
    bool entered_;

  public:
    ApiGuard()
        : entered_(host::enterApi()) {
    }
    ~ApiGuard() {
        if (entered_) {
            host::leaveApi();
        }
    }

    bool entered() const {
        return entered_;
    }
};

extern "C" J2534_LONG J2534_CALL PassThruScanForDevices(J2534_ULONG *pDeviceCount) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruScanForDevices",
                                "count_out=%p",
                                pDeviceCount);
    trace.outputValue("device_count", pDeviceCount);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruScanForDevices(pDeviceCount)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruGetNextDevice(SDEVICE *psDevice) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruGetNextDevice",
                                "device_out=%p",
                                psDevice);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruGetNextDevice(psDevice)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruOpen(const char *pName, J2534_ULONG *pDeviceID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruOpen",
                                "name_ptr=%p device_out=%p",
                                pName,
                                pDeviceID);
    trace.outputValue("device_id", pDeviceID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruOpen(pName, pDeviceID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruClose(J2534_ULONG DeviceID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruClose",
                                "device=0x%08" J2534_PRIX,
                                DeviceID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruClose(DeviceID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruConnect(J2534_ULONG DeviceID,
                                                 J2534_ULONG ProtocolID,
                                                 J2534_ULONG Flags,
                                                 J2534_ULONG BaudRate,
                                                 RESOURCE_STRUCT ResourceStruct,
                                                 J2534_ULONG *pChannelID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruConnect",
                                "device=0x%08" J2534_PRIX " protocol=0x%08" J2534_PRIX
                                " flags=0x%08" J2534_PRIX " baudrate=%" J2534_PRIu " "
                                "connector=%" J2534_PRIu " pin_count=%" J2534_PRIu " pins=%p",
                                DeviceID,
                                ProtocolID,
                                Flags,
                                BaudRate,
                                ResourceStruct.Connector,
                                ResourceStruct.NumOfResources,
                                ResourceStruct.ResourceListPtr);
    trace.outputValue("channel_id", pChannelID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruConnect(DeviceID,
                                                                  ProtocolID,
                                                                  Flags,
                                                                  BaudRate,
                                                                  ResourceStruct,
                                                                  pChannelID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruDisconnect(J2534_ULONG ChannelID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruDisconnect",
                                "channel=0x%08" J2534_PRIX,
                                ChannelID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruDisconnect(ChannelID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

static J2534_LONG logicalConnect(J2534_ULONG PhysicalChannelID,
                                 J2534_ULONG ProtocolID,
                                 J2534_ULONG Flags,
                                 void *pChannelDescriptor,
                                 J2534_ULONG *pChannelID,
                                 bool legacyChannel) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                legacyChannel ? "OpenDiagLogicalConnect0404"
                                              : "PassThruLogicalConnect",
                                "physical=0x%08" J2534_PRIX " protocol=0x%08" J2534_PRIX
                                " flags=0x%08" J2534_PRIX " descriptor=%p",
                                PhysicalChannelID,
                                ProtocolID,
                                Flags,
                                pChannelDescriptor);
    trace.outputValue("channel_id", pChannelID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruLogicalConnect(PhysicalChannelID,
                                                                         ProtocolID,
                                                                         Flags,
                                                                         pChannelDescriptor,
                                                                         pChannelID,
                                                                         legacyChannel)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruLogicalConnect(J2534_ULONG physical,
                                                        J2534_ULONG protocol,
                                                        J2534_ULONG flags,
                                                        void *descriptor,
                                                        J2534_ULONG *channel) {
    return logicalConnect(physical, protocol, flags, descriptor, channel, false);
}

// Private bridge: all 04.04 ISO-TP peers belong to one public receive channel.
extern "C" J2534_EXPORT J2534_LONG J2534_CALL OpenDiagLogicalConnect0404(J2534_ULONG physical,
                                                                         J2534_ULONG protocol,
                                                                         J2534_ULONG flags,
                                                                         void *descriptor,
                                                                         J2534_ULONG *channel) {
    return logicalConnect(physical, protocol, flags, descriptor, channel, true);
}

extern "C" J2534_LONG J2534_CALL PassThruLogicalDisconnect(J2534_ULONG ChannelID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruLogicalDisconnect",
                                "channel=0x%08" J2534_PRIX,
                                ChannelID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruLogicalDisconnect(ChannelID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruSelect(SCHANNELSET *ChannelSetPtr,
                                                J2534_ULONG SelectType,
                                                J2534_ULONG Timeout) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruSelect",
                                "channels=%p type=%" J2534_PRIu " timeout_ms=%" J2534_PRIu,
                                ChannelSetPtr,
                                SelectType,
                                Timeout);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(
            host::finishApi(host::PassThruSelect(ChannelSetPtr, SelectType, Timeout)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruReadMsgs(J2534_ULONG ChannelID,
                                                  PASSTHRU_MSG *pMsg,
                                                  J2534_ULONG *pNumMsgs,
                                                  J2534_ULONG Timeout) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruReadMsgs",
                                "channel=0x%08" J2534_PRIX " messages=%p timeout_ms=%" J2534_PRIu,
                                ChannelID,
                                pMsg,
                                Timeout);
    trace.inputValue("requested_count", pNumMsgs);
    trace.outputValue("returned_count", pNumMsgs, true);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(
            host::finishApi(host::PassThruReadMsgs(ChannelID, pMsg, pNumMsgs, Timeout)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruQueueMsgs(J2534_ULONG ChannelID,
                                                   PASSTHRU_MSG *pMsg,
                                                   J2534_ULONG *pNumMsgs) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruQueueMsgs",
                                "channel=0x%08" J2534_PRIX " messages=%p",
                                ChannelID,
                                pMsg);
    trace.inputValue("requested_count", pNumMsgs);
    trace.outputValue("returned_count", pNumMsgs, true);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruQueueMsgs(ChannelID, pMsg, pNumMsgs)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruStartPeriodicMsg(J2534_ULONG ChannelID,
                                                          PASSTHRU_MSG *pMsg,
                                                          J2534_ULONG *pMsgID,
                                                          J2534_ULONG TimeInterval) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruStartPeriodicMsg",
                                "channel=0x%08" J2534_PRIX " message=%p interval_ms=%" J2534_PRIu,
                                ChannelID,
                                pMsg,
                                TimeInterval);
    trace.outputValue("periodic_id", pMsgID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(
            host::finishApi(host::PassThruStartPeriodicMsg(ChannelID, pMsg, pMsgID, TimeInterval)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruStopPeriodicMsg(J2534_ULONG ChannelID, J2534_ULONG MsgID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruStopPeriodicMsg",
                                "channel=0x%08" J2534_PRIX " message_id=0x%08" J2534_PRIX,
                                ChannelID,
                                MsgID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruStopPeriodicMsg(ChannelID, MsgID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruStartMsgFilter(J2534_ULONG ChannelID,
                                                        J2534_ULONG FilterType,
                                                        PASSTHRU_MSG *pMaskMsg,
                                                        PASSTHRU_MSG *pPatternMsg,
                                                        J2534_ULONG *pFilterID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruStartMsgFilter",
                                "channel=0x%08" J2534_PRIX " type=%" J2534_PRIu
                                " mask=%p pattern=%p",
                                ChannelID,
                                FilterType,
                                pMaskMsg,
                                pPatternMsg);
    trace.outputValue("filter_id", pFilterID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(
            host::PassThruStartMsgFilter(ChannelID, FilterType, pMaskMsg, pPatternMsg, pFilterID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruStopMsgFilter(J2534_ULONG ChannelID,
                                                       J2534_ULONG FilterID) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruStopMsgFilter",
                                "channel=0x%08" J2534_PRIX " filter_id=0x%08" J2534_PRIX,
                                ChannelID,
                                FilterID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(host::PassThruStopMsgFilter(ChannelID, FilterID)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruSetProgrammingVoltage(J2534_ULONG DeviceID,
                                                               RESOURCE_STRUCT ResourceStruct,
                                                               J2534_ULONG Voltage) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruSetProgrammingVoltage",
                                "device=0x%08" J2534_PRIX " connector=%" J2534_PRIu
                                " pin_count=%" J2534_PRIu " pins=%p millivolts=%" J2534_PRIu,
                                DeviceID,
                                ResourceStruct.Connector,
                                ResourceStruct.NumOfResources,
                                ResourceStruct.ResourceListPtr,
                                Voltage);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(
            host::PassThruSetProgrammingVoltage(DeviceID, ResourceStruct, Voltage)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruReadVersion(J2534_ULONG DeviceID,
                                                     char *pFirmwareVersion,
                                                     char *pDllVersion,
                                                     char *pApiVersion) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruReadVersion",
                                "device=0x%08" J2534_PRIX,
                                DeviceID);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::finishApi(
            host::PassThruReadVersion(DeviceID, pFirmwareVersion, pDllVersion, pApiVersion)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruGetLastError(char *pErrorDescription) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruGetLastError",
                                "description=%p",
                                pErrorDescription);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(host::PassThruGetLastError(pErrorDescription));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruIoctl(J2534_ULONG ControlTarget,
                                               J2534_ULONG IoctlID,
                                               void *InputPtr,
                                               void *OutputPtr) {
    diagnostics::ApiTrace trace(moduleAnchor,
                                "05.00",
                                "PassThruIoctl",
                                "target=0x%08" J2534_PRIX " ioctl=0x%08" J2534_PRIX
                                " input=%p output=%p",
                                ControlTarget,
                                IoctlID,
                                InputPtr,
                                OutputPtr);

    ApiGuard guard;
    if (!guard.entered()) {
        return trace.result(ERR_CONCURRENT_API_CALL);
    }

    try {
        return trace.result(
            host::finishApi(host::PassThruIoctl(ControlTarget, IoctlID, InputPtr, OutputPtr)));
    } catch (const std::exception &error) {
        return trace.result(host::finishApi(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(host::finishApi(ERR_FAILED, "Unexpected host exception"));
    }
}
