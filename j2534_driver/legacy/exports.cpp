// SPDX-License-Identifier: GPL-3.0-only
#include "../common/api_trace.h"
#include "legacy.h"
#include "../common/export_call.h"
#include <exception>

extern "C" J2534_LONG J2534_CALL PassThruOpen(void *name, J2534_ULONG *deviceId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruOpen",
                                "name_ptr=%p device_out=%p",
                                name,
                                deviceId);
    trace.outputValue("device_id", deviceId);

    legacy::ApiGuard guard;
    return runExport(trace, [&]() { return legacy::PassThruOpen(name, deviceId); }, legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruClose(J2534_ULONG deviceId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruClose",
                                "device=0x%08" J2534_PRIX,
                                deviceId);

    legacy::ApiGuard guard;
    return runExport(trace, [&]() { return legacy::PassThruClose(deviceId); }, legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruConnect(J2534_ULONG deviceId,
                                                 J2534_ULONG protocol,
                                                 J2534_ULONG flags,
                                                 J2534_ULONG baudrate,
                                                 J2534_ULONG *channelId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruConnect",
                                "device=0x%08" J2534_PRIX " protocol=0x%08" J2534_PRIX
                                " flags=0x%08" J2534_PRIX " baudrate=%" J2534_PRIu
                                " channel_out=%p",
                                deviceId,
                                protocol,
                                flags,
                                baudrate,
                                channelId);
    trace.outputValue("channel_id", channelId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruConnect(deviceId, protocol, flags, baudrate, channelId); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruDisconnect(J2534_ULONG channelId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruDisconnect",
                                "channel=0x%08" J2534_PRIX,
                                channelId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruDisconnect(channelId); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruReadMsgs(J2534_ULONG channelId,
                                                  PASSTHRU_MSG *messages,
                                                  J2534_ULONG *count,
                                                  J2534_ULONG timeout) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruReadMsgs",
                                "channel=0x%08" J2534_PRIX " messages=%p timeout_ms=%" J2534_PRIu,
                                channelId,
                                messages,
                                timeout);
    trace.inputValue("requested_count", count);
    trace.outputValue("returned_count", count, true);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruReadMsgs(channelId, messages, count, timeout); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruWriteMsgs(J2534_ULONG channelId,
                                                   PASSTHRU_MSG *messages,
                                                   J2534_ULONG *count,
                                                   J2534_ULONG timeout) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruWriteMsgs",
                                "channel=0x%08" J2534_PRIX " messages=%p timeout_ms=%" J2534_PRIu,
                                channelId,
                                messages,
                                timeout);
    trace.inputValue("requested_count", count);
    trace.outputValue("returned_count", count, true);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruWriteMsgs(channelId, messages, count, timeout); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruStartPeriodicMsg(J2534_ULONG channelId,
                                                          PASSTHRU_MSG *message,
                                                          J2534_ULONG *messageId,
                                                          J2534_ULONG interval) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruStartPeriodicMsg",
                                "channel=0x%08" J2534_PRIX " message=%p interval_ms=%" J2534_PRIu,
                                channelId,
                                message,
                                interval);
    trace.outputValue("periodic_id", messageId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruStartPeriodicMsg(channelId, message, messageId, interval); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruStopPeriodicMsg(J2534_ULONG channelId,
                                                         J2534_ULONG messageId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruStopPeriodicMsg",
                                "channel=0x%08" J2534_PRIX " message_id=0x%08" J2534_PRIX,
                                channelId,
                                messageId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruStopPeriodicMsg(channelId, messageId); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruStartMsgFilter(J2534_ULONG channelId,
                                                        J2534_ULONG type,
                                                        PASSTHRU_MSG *mask,
                                                        PASSTHRU_MSG *pattern,
                                                        PASSTHRU_MSG *flow,
                                                        J2534_ULONG *filterId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruStartMsgFilter",
                                "channel=0x%08" J2534_PRIX " type=%" J2534_PRIu
                                " mask=%p pattern=%p flow=%p",
                                channelId,
                                type,
                                mask,
                                pattern,
                                flow);
    trace.outputValue("filter_id", filterId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() {
            return legacy::PassThruStartMsgFilter(channelId, type, mask, pattern, flow, filterId);
        },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruStopMsgFilter(J2534_ULONG channelId,
                                                       J2534_ULONG filterId) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruStopMsgFilter",
                                "channel=0x%08" J2534_PRIX " filter_id=0x%08" J2534_PRIX,
                                channelId,
                                filterId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruStopMsgFilter(channelId, filterId); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruSetProgrammingVoltage(J2534_ULONG deviceId,
                                                               J2534_ULONG pin,
                                                               J2534_ULONG voltage) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruSetProgrammingVoltage",
                                "device=0x%08" J2534_PRIX " pin=%" J2534_PRIu
                                " millivolts=%" J2534_PRIu,
                                deviceId,
                                pin,
                                voltage);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruSetProgrammingVoltage(deviceId, pin, voltage); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruReadVersion(J2534_ULONG deviceId,
                                                     char *firmware,
                                                     char *dll,
                                                     char *api) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruReadVersion",
                                "device=0x%08" J2534_PRIX,
                                deviceId);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruReadVersion(deviceId, firmware, dll, api); },
        legacy::finish);
}

extern "C" J2534_LONG J2534_CALL PassThruGetLastError(char *description) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruGetLastError",
                                "description=%p",
                                description);

    legacy::ApiGuard guard;
    try {
        return trace.result(legacy::PassThruGetLastError(description));
    } catch (const std::exception &error) {
        return trace.result(legacy::finish(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(legacy::finish(ERR_FAILED, "Unexpected host exception"));
    }
}

extern "C" J2534_LONG J2534_CALL PassThruIoctl(J2534_ULONG target,
                                               J2534_ULONG id,
                                               void *input,
                                               void *output) {
    diagnostics::ApiTrace trace(legacyAnchor,
                                "04.04",
                                "PassThruIoctl",
                                "target=0x%08" J2534_PRIX " ioctl=0x%08" J2534_PRIX
                                " input=%p output=%p",
                                target,
                                id,
                                input,
                                output);

    legacy::ApiGuard guard;
    return runExport(
        trace,
        [&]() { return legacy::PassThruIoctl(target, id, input, output); },
        legacy::finish);
}
