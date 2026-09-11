// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../include/j2534_0404.h"
#include "../platform/platform.h"

extern const void *legacyAnchor;

namespace legacy {
bool enter();
void leave();
J2534_LONG finish(J2534_LONG status, const char *detail = NULL);

class ApiGuard {
    bool entered_;

  public:
    ApiGuard()
        : entered_(enter()) {
    }
    ~ApiGuard() {
        if (entered_) {
            leave();
        }
    }
    bool entered() const {
        return entered_;
    }
};
J2534_LONG PassThruOpen(void *name, J2534_ULONG *deviceId);
J2534_LONG PassThruClose(J2534_ULONG deviceId);
J2534_LONG PassThruConnect(J2534_ULONG deviceId,
                           J2534_ULONG protocol,
                           J2534_ULONG flags,
                           J2534_ULONG baudrate,
                           J2534_ULONG *channelId);
J2534_LONG PassThruDisconnect(J2534_ULONG channelId);
J2534_LONG PassThruReadMsgs(J2534_ULONG channelId,
                            PASSTHRU_MSG *messages,
                            J2534_ULONG *count,
                            J2534_ULONG timeout);
J2534_LONG PassThruWriteMsgs(J2534_ULONG channelId,
                             PASSTHRU_MSG *messages,
                             J2534_ULONG *count,
                             J2534_ULONG timeout);
J2534_LONG PassThruStartPeriodicMsg(J2534_ULONG channelId,
                                    PASSTHRU_MSG *message,
                                    J2534_ULONG *messageId,
                                    J2534_ULONG interval);
J2534_LONG PassThruStopPeriodicMsg(J2534_ULONG channelId, J2534_ULONG messageId);
J2534_LONG PassThruStartMsgFilter(J2534_ULONG channelId,
                                  J2534_ULONG type,
                                  PASSTHRU_MSG *mask,
                                  PASSTHRU_MSG *pattern,
                                  PASSTHRU_MSG *flow,
                                  J2534_ULONG *filterId);
J2534_LONG PassThruStopMsgFilter(J2534_ULONG channelId, J2534_ULONG filterId);
J2534_LONG PassThruSetProgrammingVoltage(J2534_ULONG deviceId,
                                         J2534_ULONG pin,
                                         J2534_ULONG voltage);
J2534_LONG PassThruReadVersion(J2534_ULONG deviceId, char *firmware, char *dll, char *api);
J2534_LONG PassThruGetLastError(char *description);
J2534_LONG PassThruIoctl(J2534_ULONG target, J2534_ULONG id, void *input, void *output);
} // namespace legacy
