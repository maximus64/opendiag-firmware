// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../platform/platform.h"
#include <string>

#define OPENDIAG_TYPES_ONLY
namespace native {
#include "../include/j2534.h"
}
#undef OPENDIAG_TYPES_ONLY

class Backend {
    platform::Library library_;

  public:
    Backend()
        : library_(NULL) {
    }
    void load(const void *owner);
    void unload();
    typedef J2534_LONG(J2534_CALL *PassThruScanForDevicesFn)(J2534_ULONG *pDeviceCount);
    PassThruScanForDevicesFn scanForDevices;
    typedef J2534_LONG(J2534_CALL *PassThruGetNextDeviceFn)(native::SDEVICE *psDevice);
    PassThruGetNextDeviceFn getNextDevice;
    typedef J2534_LONG(J2534_CALL *PassThruOpenFn)(const char *pName, J2534_ULONG *pDeviceID);
    PassThruOpenFn open;
    typedef J2534_LONG(J2534_CALL *PassThruCloseFn)(J2534_ULONG DeviceID);
    PassThruCloseFn close;
    typedef J2534_LONG(J2534_CALL *PassThruConnectFn)(J2534_ULONG DeviceID,
                                                      J2534_ULONG ProtocolID,
                                                      J2534_ULONG Flags,
                                                      J2534_ULONG BaudRate,
                                                      native::RESOURCE_STRUCT ResourceStruct,
                                                      J2534_ULONG *pChannelID);
    PassThruConnectFn connect;
    typedef J2534_LONG(J2534_CALL *PassThruDisconnectFn)(J2534_ULONG ChannelID);
    PassThruDisconnectFn disconnect;
    typedef J2534_LONG(J2534_CALL *PassThruLogicalConnectFn)(J2534_ULONG PhysicalChannelID,
                                                             J2534_ULONG ProtocolID,
                                                             J2534_ULONG Flags,
                                                             void *pChannelDescriptor,
                                                             J2534_ULONG *pChannelID);
    PassThruLogicalConnectFn logicalConnect;
    typedef J2534_LONG(J2534_CALL *PassThruLogicalDisconnectFn)(J2534_ULONG ChannelID);
    PassThruLogicalDisconnectFn logicalDisconnect;
    typedef J2534_LONG(J2534_CALL *PassThruSelectFn)(native::SCHANNELSET *ChannelSetPtr,
                                                     J2534_ULONG SelectType,
                                                     J2534_ULONG Timeout);
    PassThruSelectFn select;
    typedef J2534_LONG(J2534_CALL *PassThruReadMsgsFn)(J2534_ULONG ChannelID,
                                                       native::PASSTHRU_MSG *pMsg,
                                                       J2534_ULONG *pNumMsgs,
                                                       J2534_ULONG Timeout);
    PassThruReadMsgsFn readMsgs;
    typedef J2534_LONG(J2534_CALL *PassThruQueueMsgsFn)(J2534_ULONG ChannelID,
                                                        native::PASSTHRU_MSG *pMsg,
                                                        J2534_ULONG *pNumMsgs);
    PassThruQueueMsgsFn queueMsgs;
    typedef J2534_LONG(J2534_CALL *PassThruStartPeriodicMsgFn)(J2534_ULONG ChannelID,
                                                               native::PASSTHRU_MSG *pMsg,
                                                               J2534_ULONG *pMsgID,
                                                               J2534_ULONG TimeInterval);
    PassThruStartPeriodicMsgFn startPeriodicMsg;
    typedef J2534_LONG(J2534_CALL *PassThruStopPeriodicMsgFn)(J2534_ULONG ChannelID,
                                                              J2534_ULONG MsgID);
    PassThruStopPeriodicMsgFn stopPeriodicMsg;
    typedef J2534_LONG(J2534_CALL *PassThruStartMsgFilterFn)(J2534_ULONG ChannelID,
                                                             J2534_ULONG FilterType,
                                                             native::PASSTHRU_MSG *pMaskMsg,
                                                             native::PASSTHRU_MSG *pPatternMsg,
                                                             J2534_ULONG *pFilterID);
    PassThruStartMsgFilterFn startMsgFilter;
    typedef J2534_LONG(J2534_CALL *PassThruStopMsgFilterFn)(J2534_ULONG ChannelID,
                                                            J2534_ULONG FilterID);
    PassThruStopMsgFilterFn stopMsgFilter;
    typedef J2534_LONG(J2534_CALL *PassThruSetProgrammingVoltageFn)(
        J2534_ULONG DeviceID,
        native::RESOURCE_STRUCT ResourceStruct,
        J2534_ULONG Voltage);
    PassThruSetProgrammingVoltageFn setProgrammingVoltage;
    typedef J2534_LONG(J2534_CALL *PassThruReadVersionFn)(J2534_ULONG DeviceID,
                                                          char *pFirmwareVersion,
                                                          char *pDllVersion,
                                                          char *pApiVersion);
    PassThruReadVersionFn readVersion;
    typedef J2534_LONG(J2534_CALL *PassThruGetLastErrorFn)(char *pErrorDescription);
    PassThruGetLastErrorFn getLastError;
    typedef J2534_LONG(J2534_CALL *PassThruIoctlFn)(J2534_ULONG ControlTarget,
                                                    J2534_ULONG IoctlID,
                                                    void *InputPtr,
                                                    void *OutputPtr);
    PassThruIoctlFn ioctl;
};
