// SPDX-License-Identifier: GPL-3.0-only
#include "../include/j2534.h"
#include "link.h"
#include <algorithm>
#include <ctype.h>
#include <map>
#include <stdexcept>
#include <stdio.h>
#include <vector>

namespace host {

struct Channel {
    J2534_ULONG remote;
    J2534_ULONG protocol;
    J2534_ULONG parent;
    std::map<J2534_ULONG, J2534_ULONG> filters;
    std::map<J2534_ULONG, J2534_ULONG> periodic;
    std::map<J2534_ULONG, J2534_ULONG> repeats;
};

struct Device {
    Link link;
    Settings settings;
    J2534_ULONG id;
    J2534_ULONG remote;
    opendiag_Capabilities capabilities;
    bool disconnected;
    bool programmingVoltage;
    platform::Lease lease;
    std::map<J2534_ULONG, Channel> channels;

    Device()
        : id(0),
          remote(0),
          capabilities(),
          disconnected(false),
          programmingVoltage(false) {
    }
    ~Device() {
        link.close();
    }
};

static Device *device;
static J2534_ULONG nextHandle = 0x100;
static SDEVICE scannedDevice;
static bool scanFound;
static bool scanConsumed;
static std::atomic<bool> apiBusy(false);
static char lastError[80] = "No error";
static char detail[80];

bool enterApi() {
    if (apiBusy.exchange(true)) {
        return false;
    }

    detail[0] = 0;
    return true;
}

void leaveApi() {
    apiBusy.store(false);
}

J2534_LONG finishApi(J2534_LONG status, const char *message) {
    if (message) {
        platform::copyText(detail, sizeof(detail), message);
    }

    if (status == STATUS_NOERROR) {
        platform::copyText(lastError, sizeof(lastError), "No error");
    } else if (detail[0]) {
        platform::format(lastError, sizeof(lastError), "0x%02" J2534_PRIX ": %s", status, detail);
    } else {
        platform::format(lastError, sizeof(lastError), "J2534 error 0x%02" J2534_PRIX, status);
    }

    return status;
}

static J2534_ULONG newHandle() {
    ++nextHandle;
    if (nextHandle == 0) {
        ++nextHandle;
    }

    return nextHandle;
}

static bool acquireLease(Device &candidate) {
    char port[16];
    platform::format(port, sizeof(port), "%u", candidate.settings.port);
    std::string endpoint;
    if (candidate.settings.transport == "tcp") {
        endpoint = candidate.settings.host + ":" + port;
    } else {
        endpoint = candidate.settings.com;
    }

    return candidate.lease.acquire(endpoint);
}

static J2534_LONG checkDevice() {
    if (!device) {
        return ERR_DEVICE_NOT_OPEN;
    }

    if (device->disconnected) {
        return ERR_DEVICE_NOT_CONNECTED;
    }

    if (!device->link.alive()) {
        device->disconnected = true;
        device->link.close();
        return ERR_DEVICE_NOT_CONNECTED;
    }

    return STATUS_NOERROR;
}

static J2534_LONG checkDevice(J2534_ULONG id) {
    J2534_LONG status = checkDevice();
    if (status) {
        return status;
    }

    if (id != device->id) {
        return ERR_INVALID_DEVICE_ID;
    }

    return STATUS_NOERROR;
}

static J2534_LONG checkChannel(J2534_ULONG id, Channel *&channel) {
    J2534_LONG status = checkDevice();
    if (status) {
        return status;
    }

    std::map<J2534_ULONG, Channel>::iterator found = device->channels.find(id);
    if (found == device->channels.end()) {
        return ERR_INVALID_CHANNEL_ID;
    }

    channel = &found->second;
    return STATUS_NOERROR;
}

// A broken wire exchange invalidates the session; replay could duplicate a vehicle command.
static J2534_LONG rpc(const opendiag_Request &request, opendiag_Response &response) {
    memset(&response, 0, sizeof(response));
    try {
        device->link.rpc(request, response);
        if (response.status == ERR_DEVICE_NOT_OPEN || response.status == ERR_INVALID_DEVICE_ID) {
            device->disconnected = true;
            device->link.close();
            memset(&response, 0, sizeof(response));
            return ERR_DEVICE_NOT_CONNECTED;
        }
        return response.status;
    } catch (const std::exception &error) {
        memset(&response, 0, sizeof(response));
        platform::copyText(detail, sizeof(detail), error.what());
        device->disconnected = true;
        device->link.close();
        return ERR_DEVICE_NOT_CONNECTED;
    }
}

static bool compatible(Link &link, opendiag_Capabilities *capabilities = NULL) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_capabilities_tag;
    try {
        link.rpc(request, response);
    } catch (const std::exception &) {
        // Only the side-effect-free startup probe is retried.
        link.selectFrontend(true);
        link.rpc(request, response);
    }
    if (capabilities) {
        *capabilities = response.capabilities;
    }
    return !response.status && response.has_capabilities &&
           response.capabilities.wire_version == 1 && response.capabilities.api_version == 0x0500 &&
           response.capabilities.fragment_bytes == 192 && response.capabilities.rpc_bytes == 4608;
}

J2534_LONG PassThruScanForDevices(J2534_ULONG *count) {
    if (!count) {
        return ERR_NULL_PARAMETER;
    }

    Device candidate;
    candidate.settings = loadSettings();
    SDEVICE info = {};
    platform::copyText(info.DeviceName, sizeof(info.DeviceName), candidate.settings.name.c_str());
    info.DeviceConnectMedia = DEVICE_CONN_WIRED;
    info.DeviceSignalQuality = 0xffffffffUL;
    info.DeviceSignalStrength = 0xffffffffUL;
    info.DeviceConnectSpeed = candidate.settings.transport == "com" ? 12000000 : 0;
    scanFound = false;
    scanConsumed = false;
    if (device || !acquireLease(candidate)) {
        info.DeviceAvailable = DEVICE_IN_USE;
        info.DeviceDLLFWStatus = device && !device->disconnected
                                     ? DEVICE_DLL_FW_COMPATIBLE
                                     : DEVICE_DLL_FW_COMPATIBILTY_UNKNOWN;
        scanFound = true;
    } else {
        try {
            candidate.link.open(candidate.settings);
            info.DeviceDLLFWStatus = compatible(candidate.link) ? DEVICE_DLL_FW_COMPATIBLE
                                                                : DEVICE_DLL_OR_FW_NOT_COMPATIBLE;
            info.DeviceAvailable = DEVICE_AVAILABLE;
            scanFound = true;
        } catch (const std::exception &) {
            scanFound = false;
        }
    }

    scannedDevice = info;
    *count = scanFound ? 1 : 0;
    return STATUS_NOERROR;
}

J2534_LONG PassThruGetNextDevice(SDEVICE *output) {
    if (!output) {
        return ERR_NULL_PARAMETER;
    }

    if (!scanFound) {
        return ERR_BUFFER_EMPTY;
    }

    if (scanConsumed) {
        return ERR_EXCEEDED_LIMIT;
    }

    *output = scannedDevice;
    scanConsumed = true;
    return STATUS_NOERROR;
}

J2534_LONG PassThruOpen(const char *name, J2534_ULONG *id) {
    if (!name || !id) {
        return ERR_NULL_PARAMETER;
    }

    if (device) {
        return device->disconnected ? ERR_DEVICE_NOT_CONNECTED : ERR_DEVICE_IN_USE;
    }

    Device *candidate = new Device;
    J2534_LONG failure = ERR_OPEN_FAILED;
    try {
        candidate->settings = loadSettings();
        std::string expected = "J2534-1:" + candidate->settings.name;
        if (strcmp(name, expected.c_str()) && strcmp(name, "J2534-1:")) {
            delete candidate;
            return ERR_OPEN_FAILED;
        }
        if (!acquireLease(*candidate)) {
            delete candidate;
            return ERR_DEVICE_IN_USE;
        }
        failure = ERR_DEVICE_NOT_CONNECTED;
        candidate->link.open(candidate->settings);
        if (!compatible(candidate->link, &candidate->capabilities)) {
            delete candidate;
            return ERR_OPEN_FAILED;
        }
        opendiag_Request request = {};
        opendiag_Response response = {};
        request.which_command = opendiag_Request_open_tag;
        candidate->link.rpc(request, response);
        if (response.status || !response.id) {
            J2534_LONG status = response.status ? response.status : ERR_OPEN_FAILED;
            delete candidate;
            return status;
        }
        candidate->id = newHandle();
        candidate->remote = response.id;
        device = candidate;
        *id = device->id;
        return STATUS_NOERROR;
    } catch (const std::exception &error) {
        platform::copyText(detail, sizeof(detail), error.what());
        delete candidate;
        return failure;
    }
}

J2534_LONG PassThruClose(J2534_ULONG id) {
    if (!device) {
        return ERR_DEVICE_NOT_OPEN;
    }

    if (id != device->id) {
        return ERR_INVALID_DEVICE_ID;
    }

    J2534_LONG status = checkDevice(id);
    if (!status) {
        opendiag_Request request = {};
        opendiag_Response response = {};
        request.which_command = opendiag_Request_close_tag;
        request.command.close.id = device->remote;
        status = rpc(request, response);
        if (status && status != ERR_DEVICE_NOT_CONNECTED) {
            return status;
        }
    }

    delete device;
    device = NULL;
    return status;
}

J2534_LONG PassThruConnect(J2534_ULONG id,
                           J2534_ULONG protocol,
                           J2534_ULONG flags,
                           J2534_ULONG baud,
                           RESOURCE_STRUCT resources,
                           J2534_ULONG *channelId) {
    J2534_LONG status = checkDevice(id);
    if (status) {
        return status;
    }

    if (!channelId || !resources.ResourceListPtr) {
        return ERR_NULL_PARAMETER;
    }

    if (resources.NumOfResources > 2 || !resources.NumOfResources) {
        return ERR_PIN_NOT_SUPPORTED;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_connect_tag;
    opendiag_Connect &connect = request.command.connect;
    connect.device = device->remote;
    connect.protocol = protocol;
    connect.flags = flags;
    connect.baudrate = baud;
    connect.connector = resources.Connector;
    connect.pins_count = (pb_size_t)resources.NumOfResources;
    for (unsigned i = 0; i < connect.pins_count; ++i) {
        connect.pins[i] = resources.ResourceListPtr[i];
    }

    status = rpc(request, response);
    if (status) {
        return status;
    }

    Channel channel = {};
    channel.remote = response.id;
    channel.protocol = protocol;
    J2534_ULONG handle = newHandle();
    device->channels[handle] = channel;
    *channelId = handle;
    return STATUS_NOERROR;
}

J2534_LONG PassThruLogicalConnect(J2534_ULONG physical,
                                  J2534_ULONG protocol,
                                  J2534_ULONG flags,
                                  void *descriptor,
                                  J2534_ULONG *channelId,
                                  bool legacyChannel) {
    Channel *parent = NULL;
    J2534_LONG status = checkChannel(physical, parent);
    if (status) {
        return status;
    }

    if (!descriptor || !channelId) {
        return ERR_NULL_PARAMETER;
    }

    if (parent->parent || parent->protocol != CAN || protocol != ISO15765_LOGICAL) {
        return ERR_LOG_CHAN_NOT_ALLOWED;
    }

    ISO15765_CHANNEL_DESCRIPTOR &desc = *(ISO15765_CHANNEL_DESCRIPTOR *)descriptor;
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_logical_connect_tag;
    opendiag_LogicalConnect &connect = request.command.logical_connect;
    connect.physical = parent->remote;
    connect.protocol = protocol;
    connect.flags = flags;
    connect.legacy_channel = legacyChannel;
    connect.local_flags = desc.LocalTxFlags;
    connect.remote_flags = desc.RemoteTxFlags;
    connect.local_address.size = desc.LocalTxFlags & ISO15765_ADDR_TYPE ? 5 : 4;
    connect.remote_address.size = desc.RemoteTxFlags & ISO15765_ADDR_TYPE ? 5 : 4;
    memcpy(connect.local_address.bytes, desc.LocalAddress, connect.local_address.size);
    memcpy(connect.remote_address.bytes, desc.RemoteAddress, connect.remote_address.size);
    status = rpc(request, response);
    if (status) {
        return status;
    }

    Channel channel = {};
    channel.remote = response.id;
    channel.protocol = protocol;
    channel.parent = physical;
    J2534_ULONG handle = newHandle();
    device->channels[handle] = channel;
    *channelId = handle;
    return STATUS_NOERROR;
}

static J2534_LONG disconnectChannel(J2534_ULONG id, bool logical) {
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    if ((channel->parent != 0) != logical) {
        return ERR_INVALID_CHANNEL_ID;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    if (logical) {
        request.which_command = opendiag_Request_logical_disconnect_tag;
        request.command.logical_disconnect.id = channel->remote;
    } else {
        request.which_command = opendiag_Request_disconnect_tag;
        request.command.disconnect.id = channel->remote;
    }

    status = rpc(request, response);
    if (!status) {
        for (std::map<J2534_ULONG, Channel>::iterator it = device->channels.begin();
             it != device->channels.end();) {
            if (it->first == id || it->second.parent == id) {
                std::map<J2534_ULONG, Channel>::iterator removed = it;
                ++it;
                device->channels.erase(removed);
            } else {
                ++it;
            }
        }
    }

    return status;
}

J2534_LONG PassThruDisconnect(J2534_ULONG id) {
    return disconnectChannel(id, false);
}

J2534_LONG PassThruLogicalDisconnect(J2534_ULONG id) {
    return disconnectChannel(id, true);
}

static J2534_LONG encodeMessage(const PASSTHRU_MSG &input,
                                const Channel &channel,
                                opendiag_Message &output) {
    if (input.ProtocolID != channel.protocol) {
        return ERR_MSG_PROTOCOL_ID;
    }

    if (!input.DataBuffer) {
        return ERR_NULL_PARAMETER;
    }

    if (!input.DataLength || input.DataLength > input.DataBufferSize ||
        input.DataLength > sizeof(output.data.bytes)) {
        return ERR_INVALID_MSG;
    }

    output.protocol = input.ProtocolID;
    output.handle = input.MsgHandle;
    output.tx_flags = input.TxFlags;
    output.data.size = (pb_size_t)input.DataLength;
    memcpy(output.data.bytes, input.DataBuffer, input.DataLength);
    return STATUS_NOERROR;
}

static bool copyMessage(const opendiag_Message &input, PASSTHRU_MSG &output) {
    J2534_ULONG count = std::min((J2534_ULONG)input.data.size, output.DataBufferSize);
    output.ProtocolID = input.protocol;
    output.MsgHandle = input.handle;
    output.RxStatus = input.rx_status;
    output.TxFlags = input.tx_flags;
    output.Timestamp = input.timestamp_us;
    output.DataLength = count;
    output.ExtraDataIndex = std::min((J2534_ULONG)input.extra_data_index, count);
    if (count) {
        memcpy(output.DataBuffer, input.data.bytes, count);
    }

    return count == input.data.size;
}

J2534_LONG PassThruQueueMsgs(J2534_ULONG id, PASSTHRU_MSG *messages, J2534_ULONG *count) {
    if (!count) {
        return ERR_NULL_PARAMETER;
    }

    J2534_ULONG requested = *count;
    *count = 0;
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    if (!messages) {
        return ERR_NULL_PARAMETER;
    }

    if (requested > 0xffffffffUL / sizeof(PASSTHRU_MSG)) {
        return ERR_EXCEEDED_LIMIT;
    }

    for (J2534_ULONG i = 0; i < requested; ++i) {
        opendiag_Request request = {};
        opendiag_Response response = {};
        request.which_command = opendiag_Request_queue_tag;
        request.command.queue.channel = channel->remote;
        request.command.queue.has_message = true;
        status = encodeMessage(messages[i], *channel, request.command.queue.message);
        if (!status) {
            status = rpc(request, response);
        }
        if (status) {
            return status;
        }
        *count += 1;
    }

    return STATUS_NOERROR;
}

J2534_LONG PassThruReadMsgs(J2534_ULONG id,
                            PASSTHRU_MSG *messages,
                            J2534_ULONG *count,
                            J2534_ULONG timeout) {
    if (!count) {
        return ERR_NULL_PARAMETER;
    }

    J2534_ULONG requested = *count;
    *count = 0;
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    if (!messages) {
        return ERR_NULL_PARAMETER;
    }

    if (timeout > 30000) {
        return ERR_IOCTL_VALUE_NOT_SUPPORTED;
    }

    if (requested > 0xffffffffUL / sizeof(PASSTHRU_MSG)) {
        return ERR_EXCEEDED_LIMIT;
    }

    for (J2534_ULONG i = 0; i < requested; ++i) {
        if (!messages[i].DataBuffer) {
            return ERR_NULL_PARAMETER;
        }
    }

    if (!requested) {
        return STATUS_NOERROR;
    }

    // Keep caller outputs untouched until the read result permits returning messages.
    uint32_t start = platform::milliseconds();
    std::vector<opendiag_Message> collected;
    while (collected.size() < requested) {
        opendiag_Request request = {};
        opendiag_Response response = {};
        request.which_command = opendiag_Request_read_tag;
        request.command.read.id = channel->remote;
        status = rpc(request, response);
        if (status == ERR_BUFFER_EMPTY) {
            if (!timeout || platform::milliseconds() - start >= timeout) {
                if (collected.empty()) {
                    status = ERR_BUFFER_EMPTY;
                } else if (timeout) {
                    status = ERR_TIMEOUT;
                } else {
                    status = STATUS_NOERROR;
                }
                break;
            }
            platform::sleepMs(1);
            continue;
        }
        if (status) {
            return status;
        }
        if (!response.has_message || response.count != 1) {
            return ERR_INVALID_MSG;
        }
        // The small-buffer rule specifically requires a truncated copy and discards the suffix.
        if (response.message.data.size > messages[collected.size()].DataBufferSize) {
            copyMessage(response.message, messages[collected.size()]);
            return ERR_BUFFER_TOO_SMALL;
        }
        collected.push_back(response.message);
        if (response.message.rx_status & BUFFER_OVERFLOW) {
            status = ERR_BUFFER_OVERFLOW;
            break;
        }
        if (timeout && platform::milliseconds() - start >= timeout &&
            collected.size() < requested) {
            status = ERR_TIMEOUT;
            break;
        }
    }

    for (size_t i = 0; i < collected.size(); ++i) {
        copyMessage(collected[i], messages[i]);
    }

    *count = (J2534_ULONG)collected.size();
    return status;
}

J2534_LONG PassThruSelect(SCHANNELSET *set, J2534_ULONG type, J2534_ULONG timeout) {
    J2534_LONG status = checkDevice();
    if (status) {
        return status;
    }

    if (!set || !set->ChannelList) {
        return ERR_NULL_PARAMETER;
    }

    if (type != READABLE_TYPE) {
        return ERR_SELECT_TYPE_NOT_SUPPORTED;
    }

    if (timeout > 30000) {
        return ERR_IOCTL_VALUE_NOT_SUPPORTED;
    }

    if (set->ChannelCount > 5 || set->ChannelThreshold > set->ChannelCount) {
        return ERR_EXCEEDED_LIMIT;
    }

    opendiag_Request request = {};
    request.which_command = opendiag_Request_select_tag;
    request.command.select.type = type;
    request.command.select.channels_count = (pb_size_t)set->ChannelCount;
    std::vector<J2534_ULONG> original;
    for (unsigned i = 0; i < set->ChannelCount; ++i) {
        Channel *channel = NULL;
        status = checkChannel(set->ChannelList[i], channel);
        if (status) {
            return status;
        }
        if (std::find(original.begin(), original.end(), set->ChannelList[i]) != original.end()) {
            return ERR_INVALID_CHANNEL_ID;
        }
        original.push_back(set->ChannelList[i]);
        request.command.select.channels[i] = channel->remote;
    }

    uint32_t start = platform::milliseconds();
    for (;;) {
        opendiag_Response response = {};
        status = rpc(request, response);
        if (status && status != ERR_BUFFER_EMPTY) {
            return status;
        }
        if (response.channels_count > original.size()) {
            return ERR_INVALID_MSG;
        }
        J2534_ULONG readable[5] = {};
        for (unsigned i = 0; i < response.channels_count; ++i) {
            unsigned j = 0;
            while (j < original.size() &&
                   request.command.select.channels[j] != response.channels[i]) {
                ++j;
            }
            if (j == original.size() ||
                std::find(readable, readable + i, original[j]) != readable + i) {
                return ERR_INVALID_MSG;
            }
            readable[i] = original[j];
        }
        bool snapshot = timeout == 0 || set->ChannelThreshold == 0;
        bool thresholdReached = response.channels_count >= set->ChannelThreshold;
        bool timedOut = platform::milliseconds() - start >= timeout;
        if (snapshot || thresholdReached || timedOut) {
            std::copy(readable, readable + response.channels_count, set->ChannelList);
            bool enough = response.channels_count >= set->ChannelThreshold;
            set->ChannelCount = response.channels_count;
            if (!response.channels_count) {
                return ERR_BUFFER_EMPTY;
            }
            if (timeout && !enough) {
                return ERR_TIMEOUT;
            }
            return STATUS_NOERROR;
        }
        platform::sleepMs(1);
    }
}

J2534_LONG PassThruStartPeriodicMsg(J2534_ULONG id,
                                    PASSTHRU_MSG *message,
                                    J2534_ULONG *messageId,
                                    J2534_ULONG interval) {
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    if (!message || !messageId) {
        return ERR_NULL_PARAMETER;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_start_periodic_tag;
    request.command.start_periodic.channel = channel->remote;
    request.command.start_periodic.interval_ms = interval;
    request.command.start_periodic.has_message = true;
    status = encodeMessage(*message, *channel, request.command.start_periodic.message);
    if (!status) {
        status = rpc(request, response);
    }

    if (!status) {
        J2534_ULONG handle = newHandle();
        channel->periodic[handle] = response.id;
        *messageId = handle;
    }

    return status;
}

static J2534_LONG stopObject(J2534_ULONG id, J2534_ULONG objectId, bool filter) {
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    std::map<J2534_ULONG, J2534_ULONG> &objects = filter ? channel->filters : channel->periodic;
    std::map<J2534_ULONG, J2534_ULONG>::iterator object = objects.find(objectId);
    if (object == objects.end()) {
        return filter ? ERR_INVALID_FILTER_ID : ERR_INVALID_MSG_ID;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command =
        filter ? opendiag_Request_stop_filter_tag : opendiag_Request_stop_periodic_tag;
    opendiag_Object &target = filter ? request.command.stop_filter : request.command.stop_periodic;
    target.channel = channel->remote;
    target.id = object->second;
    status = rpc(request, response);
    if (!status) {
        objects.erase(object);
    }

    return status;
}

J2534_LONG PassThruStopPeriodicMsg(J2534_ULONG id, J2534_ULONG messageId) {
    return stopObject(id, messageId, false);
}

J2534_LONG PassThruStopMsgFilter(J2534_ULONG id, J2534_ULONG filterId) {
    return stopObject(id, filterId, true);
}

J2534_LONG PassThruStartMsgFilter(J2534_ULONG id,
                                  J2534_ULONG type,
                                  PASSTHRU_MSG *mask,
                                  PASSTHRU_MSG *pattern,
                                  J2534_ULONG *filterId) {
    Channel *channel = NULL;
    J2534_LONG status = checkChannel(id, channel);
    if (status) {
        return status;
    }

    if (!mask || !pattern || !filterId || !mask->DataBuffer || !pattern->DataBuffer) {
        return ERR_NULL_PARAMETER;
    }

    if (mask->ProtocolID != channel->protocol || pattern->ProtocolID != channel->protocol) {
        return ERR_MSG_PROTOCOL_ID;
    }

    if (mask->DataLength != pattern->DataLength || mask->TxFlags != pattern->TxFlags ||
        !mask->DataLength || mask->DataLength > 12 || mask->DataLength > mask->DataBufferSize ||
        pattern->DataLength > pattern->DataBufferSize) {
        return ERR_INVALID_MSG;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_start_filter_tag;
    opendiag_Filter &filter = request.command.start_filter;
    filter.channel = channel->remote;
    filter.type = type;
    filter.flags = mask->TxFlags;
    filter.mask.size = (pb_size_t)mask->DataLength;
    filter.pattern.size = (pb_size_t)pattern->DataLength;
    memcpy(filter.mask.bytes, mask->DataBuffer, mask->DataLength);
    memcpy(filter.pattern.bytes, pattern->DataBuffer, pattern->DataLength);
    status = rpc(request, response);
    if (!status) {
        J2534_ULONG handle = newHandle();
        channel->filters[handle] = response.id;
        *filterId = handle;
    }

    return status;
}

J2534_LONG PassThruSetProgrammingVoltage(J2534_ULONG id,
                                         RESOURCE_STRUCT resources,
                                         J2534_ULONG voltage) {
    J2534_LONG status = checkDevice(id);
    if (status) {
        return status;
    }

    if (!resources.ResourceListPtr) {
        return ERR_NULL_PARAMETER;
    }

    if (resources.NumOfResources != 1) {
        return ERR_PIN_NOT_SUPPORTED;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_voltage_tag;
    request.command.voltage.device = device->remote;
    request.command.voltage.connector = resources.Connector;
    request.command.voltage.pin = resources.ResourceListPtr[0];
    request.command.voltage.millivolts = voltage;
    status = rpc(request, response);
    if (!status && voltage != SHORT_TO_GROUND) {
        device->programmingVoltage = voltage != PIN_OFF;
    }

    return status;
}

J2534_LONG PassThruReadVersion(J2534_ULONG id, char *firmware, char *dll, char *api) {
    if (!firmware || !dll || !api) {
        return ERR_NULL_PARAMETER;
    }

    firmware[0] = 0;
    platform::copyText(dll, 80, "OpenDIAG 0.2.2");
    platform::copyText(api, 80, "05.00");
    J2534_LONG status = checkDevice(id);
    if (status) {
        return status;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_version_tag;
    request.command.version.id = device->remote;
    status = rpc(request, response);
    if (!status) {
        platform::copyText(firmware, 80, response.text);
    }

    return status;
}

J2534_LONG PassThruGetLastError(char *output) {
    if (!output) {
        return ERR_NULL_PARAMETER;
    }

    platform::copyText(output, 80, lastError);
    return STATUS_NOERROR;
}

static J2534_LONG readVoltageIoctl(J2534_ULONG target, J2534_ULONG id, void *input, void *output) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_ioctl_tag;
    opendiag_Ioctl &io = request.command.ioctl;
    io.id = id;
    J2534_LONG status = STATUS_NOERROR;

    status = checkDevice(target);
    if (status) {
        return status;
    }

    if (!output) {
        return ERR_NULL_PARAMETER;
    }

    io.target = device->remote;
    if (id == READ_PIN_VOLTAGE) {
        if (!input || !((RESOURCE_STRUCT *)input)->ResourceListPtr) {
            return ERR_NULL_PARAMETER;
        }
        RESOURCE_STRUCT &resources = *(RESOURCE_STRUCT *)input;
        if (resources.NumOfResources != 1) {
            return ERR_PIN_NOT_SUPPORTED;
        }
        io.connector = resources.Connector;
        io.pin = resources.ResourceListPtr[0];
    } else if (input) {
        return ERR_NULL_REQUIRED;
    }

    status = rpc(request, response);
    if (!status) {
        J2534_ULONG millivolts = response.millivolts;
        if (id == READ_PROG_VOLTAGE && !device->programmingVoltage) {
            millivolts = 0;
        }
        *(J2534_ULONG *)output = millivolts;
    }

    return status;
}

static J2534_LONG configureChannelIoctl(Channel &channel,
                                        J2534_ULONG id,
                                        void *input,
                                        void *output) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_ioctl_tag;
    opendiag_Ioctl &io = request.command.ioctl;
    io.id = id;
    io.target = channel.remote;
    J2534_LONG status = STATUS_NOERROR;

    if (output) {
        return ERR_NULL_REQUIRED;
    }

    if (!input) {
        return ERR_NULL_PARAMETER;
    }

    SCONFIG_LIST &list = *(SCONFIG_LIST *)input;
    if (list.NumOfParams && !list.ConfigPtr) {
        return ERR_NULL_PARAMETER;
    }

    if (list.NumOfParams > 0xffffffffUL / sizeof(SCONFIG)) {
        return ERR_EXCEEDED_LIMIT;
    }

    // Process one wire-sized batch at a time and stop at the first firmware error.
    for (J2534_ULONG offset = 0; offset < list.NumOfParams;) {
        io.config_count = (pb_size_t)std::min<J2534_ULONG>(32, list.NumOfParams - offset);
        for (unsigned i = 0; i < io.config_count; ++i) {
            io.config[i].parameter = list.ConfigPtr[offset + i].Parameter;
            io.config[i].value = id == SET_CONFIG ? list.ConfigPtr[offset + i].Value : 0;
        }
        status = rpc(request, response);
        if (response.config_count > io.config_count) {
            return ERR_INVALID_MSG;
        }
        if (id == GET_CONFIG) {
            for (unsigned i = 0; i < response.config_count; ++i) {
                list.ConfigPtr[offset + i].Value = response.config[i].value;
            }
        }
        if (status) {
            return status;
        }
        if (response.config_count != io.config_count) {
            return ERR_INVALID_MSG;
        }
        offset += io.config_count;
    }

    return STATUS_NOERROR;
}

static J2534_LONG fiveBaudInitIoctl(Channel &channel, void *input, void *output) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_ioctl_tag;
    opendiag_Ioctl &io = request.command.ioctl;
    io.id = FIVE_BAUD_INIT;
    io.target = channel.remote;
    J2534_LONG status = STATUS_NOERROR;

    if (!input || !output) {
        return ERR_NULL_PARAMETER;
    }

    SBYTE_ARRAY &in = *(SBYTE_ARRAY *)input;
    SBYTE_ARRAY &out = *(SBYTE_ARRAY *)output;
    if (!in.BytePtr || !out.BytePtr) {
        return ERR_NULL_PARAMETER;
    }

    if (in.NumOfBytes != 1 || out.NumOfBytes < 2) {
        return ERR_IOCTL_VALUE_NOT_SUPPORTED;
    }

    io.data.size = 1;
    io.data.bytes[0] = in.BytePtr[0];
    status = rpc(request, response);
    if (!status || status == ERR_INIT_FAILED) {
        out.NumOfBytes = std::min<J2534_ULONG>(2, response.data.size);
        memcpy(out.BytePtr, response.data.bytes, out.NumOfBytes);
    }

    return status;
}

static J2534_LONG fastInitIoctl(Channel &channel, void *input, void *output) {
    if (channel.protocol != ISO9141 && channel.protocol != ISO14230) {
        return ERR_IOCTL_ID_NOT_SUPPORTED;
    }
    if (!device->capabilities.fast_init_max_data) {
        return ERR_NOT_SUPPORTED;
    }
    PASSTHRU_MSG *out = (PASSTHRU_MSG *)output;
    if (out && !out->DataBuffer) {
        return ERR_NULL_PARAMETER;
    }

    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_fast_init_tag;
    opendiag_FastInit &init = request.command.fast_init;
    init.channel = channel.remote;
    init.no_response = !output;
    if (input) {
        opendiag_Message message = {};
        J2534_LONG status = encodeMessage(*(PASSTHRU_MSG *)input, channel, message);
        if (status) {
            return status;
        }
        if (message.data.size > device->capabilities.fast_init_max_data ||
            message.data.size > sizeof(init.data.bytes)) {
            return ERR_INVALID_MSG;
        }
        init.tx_flags = message.tx_flags;
        init.data.size = message.data.size;
        memcpy(init.data.bytes, message.data.bytes, init.data.size);
    }

    J2534_LONG status = rpc(request, response);
    if (!status && out) {
        if (!response.has_message || response.message.protocol != channel.protocol ||
            !response.message.data.size) {
            return ERR_FAILED;
        }
        if (!copyMessage(response.message, *out)) {
            return ERR_BUFFER_TOO_SMALL;
        }
    }
    return status;
}

static J2534_LONG updateLookupTableIoctl(Channel &channel,
                                         J2534_ULONG id,
                                         void *input,
                                         void *output) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_ioctl_tag;
    opendiag_Ioctl &io = request.command.ioctl;
    io.id = id;
    io.target = channel.remote;

    if (!input) {
        return ERR_NULL_PARAMETER;
    }

    if (output) {
        return ERR_NULL_REQUIRED;
    }

    SBYTE_ARRAY &bytes = *(SBYTE_ARRAY *)input;
    if (bytes.NumOfBytes && !bytes.BytePtr) {
        return ERR_NULL_PARAMETER;
    }

    if (bytes.NumOfBytes > sizeof(io.data.bytes)) {
        return ERR_EXCEEDED_LIMIT;
    }

    io.data.size = (pb_size_t)bytes.NumOfBytes;
    if (io.data.size) {
        memcpy(io.data.bytes, bytes.BytePtr, io.data.size);
    }

    return rpc(request, response);
}

static J2534_LONG clearChannelIoctl(Channel &channel, J2534_ULONG id, void *input, void *output) {
    opendiag_Request request = {};
    opendiag_Response response = {};
    request.which_command = opendiag_Request_ioctl_tag;
    request.command.ioctl.id = id;
    request.command.ioctl.target = channel.remote;
    J2534_LONG status = STATUS_NOERROR;

    if (id < CLEAR_TX_QUEUE || id > BUS_ON) {
        return ERR_IOCTL_ID_NOT_SUPPORTED;
    }

    if (input || output) {
        return ERR_NULL_REQUIRED;
    }

    status = rpc(request, response);
    if (!status && id == CLEAR_MSG_FILTERS) {
        channel.filters.clear();
    }

    if (!status && id == CLEAR_PERIODIC_MSGS) {
        channel.periodic.clear();
    }

    return status;
}

static const opendiag_ProtocolLimit *protocolLimit(J2534_ULONG protocol) {
    const opendiag_Capabilities &caps = device->capabilities;
    for (pb_size_t index = 0; index < caps.protocols_count; ++index) {
        if (caps.protocols[index].protocol == protocol) {
            return &caps.protocols[index];
        }
    }
    return NULL;
}

static J2534_LONG discoveryIoctl(J2534_ULONG target,
                                 J2534_ULONG operation,
                                 void *input,
                                 void *output) {
    J2534_LONG status = checkDevice(target);
    if (status) {
        return status;
    }
    if (!output || (operation == GET_PROTOCOL_INFO && !input)) {
        return ERR_NULL_PARAMETER;
    }
    if (operation == GET_DEVICE_INFO && input) {
        return ERR_NULL_REQUIRED;
    }
    SPARAM_LIST &list = *(SPARAM_LIST *)output;
    if (list.NumOfParams && !list.ParamPtr) {
        return ERR_NULL_PARAMETER;
    }
    const opendiag_ProtocolLimit *limit = NULL;
    if (operation == GET_PROTOCOL_INFO) {
        limit = protocolLimit(*(J2534_ULONG *)input);
        if (!limit) {
            return ERR_PROTOCOL_ID_NOT_SUPPORTED;
        }
    }
    for (J2534_ULONG index = 0; index < list.NumOfParams; ++index) {
        SPARAM &parameter = list.ParamPtr[index];
        parameter.Supported = 0;
        if (operation == GET_DEVICE_INFO &&
            (parameter.Value == ENTIRE_DEVICE || parameter.Value == 1)) {
            J2534_ULONG protocol = 0;
            switch (parameter.Parameter) {
            case J1850PWM_SUPPORTED:
                protocol = J1850PWM;
                break;
            case J1850VPW_SUPPORTED:
                protocol = J1850VPW;
                break;
            case ISO9141_SUPPORTED:
                protocol = ISO9141;
                break;
            case ISO14230_SUPPORTED:
                protocol = ISO14230;
                break;
            case CAN_SUPPORTED:
                protocol = CAN;
                break;
            default:
                break;
            }
            if (protocol && protocolLimit(protocol)) {
                parameter.Supported = 1;
                parameter.Value = 0x00010001; // One channel, also one simultaneous channel.
            }
        }
        if (limit && (parameter.Parameter == MAX_REPEAT_MESSAGING ||
                      parameter.Parameter == MAX_REPEAT_MESSAGING_LENGTH)) {
            parameter.Supported = 1;
            bool supported = device->capabilities.repeat_per_channel && limit->max_repeat_data;
            if (!supported) {
                parameter.Value = 0;
            } else if (parameter.Parameter == MAX_REPEAT_MESSAGING) {
                parameter.Value = device->capabilities.repeat_per_channel;
            } else {
                parameter.Value = limit->max_repeat_data;
            }
        }
    }
    return STATUS_NOERROR;
}

static J2534_LONG encodeRepeatMessage(const PASSTHRU_MSG &source,
                                      const Channel &channel,
                                      J2534_ULONG maximum,
                                      opendiag_RepeatMessage &message) {
    if (source.ProtocolID != channel.protocol) {
        return ERR_MSG_PROTOCOL_ID;
    }
    if (!source.DataBuffer) {
        return ERR_NULL_PARAMETER;
    }
    if (!source.DataLength || source.DataLength > source.DataBufferSize ||
        source.DataLength > maximum || source.DataLength > sizeof(message.data.bytes)) {
        return ERR_INVALID_MSG;
    }
    message.protocol = source.ProtocolID;
    message.handle = source.MsgHandle;
    message.tx_flags = source.TxFlags;
    message.data.size = (pb_size_t)source.DataLength;
    memcpy(message.data.bytes, source.DataBuffer, source.DataLength);
    return STATUS_NOERROR;
}

static J2534_LONG repeatIoctl(Channel &channel, J2534_ULONG operation, void *input, void *output) {
    if (!input || (operation != STOP_REPEAT_MESSAGE && !output)) {
        return ERR_NULL_PARAMETER;
    }
    if (operation == STOP_REPEAT_MESSAGE && output) {
        return ERR_NULL_REQUIRED;
    }
    const opendiag_ProtocolLimit *limit = protocolLimit(channel.protocol);
    if (!device->capabilities.repeat_per_channel || !limit || !limit->max_repeat_data) {
        return ERR_NOT_SUPPORTED;
    }
    opendiag_Request request = {};
    opendiag_Response response = {};
    if (operation == START_REPEAT_MESSAGE) {
        const REPEAT_MSG_SETUP &setup = *(REPEAT_MSG_SETUP *)input;
        request.which_command = opendiag_Request_start_repeat_tag;
        opendiag_Repeat &repeat = request.command.start_repeat;
        repeat.channel = channel.remote;
        repeat.interval_ms = setup.TimeInterval;
        repeat.condition = setup.Condition;
        repeat.has_message = repeat.has_mask = repeat.has_pattern = true;
        opendiag_RepeatMessage *messages[] = {&repeat.message, &repeat.mask, &repeat.pattern};
        for (unsigned index = 0; index < 3; ++index) {
            J2534_LONG status = encodeRepeatMessage(setup.RepeatMsgData[index],
                                                    channel,
                                                    limit->max_repeat_data,
                                                    *messages[index]);
            if (status) {
                return status;
            }
        }
        // Reserve host storage before creating state on the adapter.
        J2534_ULONG handle = newHandle();
        channel.repeats.insert(std::make_pair(handle, 0));
        J2534_LONG status = rpc(request, response);
        if (status) {
            channel.repeats.erase(handle);
            return status;
        }
        channel.repeats[handle] = response.id;
        *(J2534_ULONG *)output = handle;
        return STATUS_NOERROR;
    }
    J2534_ULONG handle = *(J2534_ULONG *)input;
    std::map<J2534_ULONG, J2534_ULONG>::iterator found = channel.repeats.find(handle);
    if (found == channel.repeats.end()) {
        return ERR_INVALID_MSG_ID;
    }
    if (operation == QUERY_REPEAT_MESSAGE) {
        request.which_command = opendiag_Request_query_repeat_tag;
        request.command.query_repeat.channel = channel.remote;
        request.command.query_repeat.id = found->second;
    } else {
        request.which_command = opendiag_Request_stop_repeat_tag;
        request.command.stop_repeat.channel = channel.remote;
        request.command.stop_repeat.id = found->second;
    }
    J2534_LONG status = rpc(request, response);
    if (!status) {
        if (operation == QUERY_REPEAT_MESSAGE) {
            *(J2534_ULONG *)output = response.repeat_active ? 1 : 0;
        } else {
            channel.repeats.erase(found);
        }
    }
    return status;
}

J2534_LONG PassThruIoctl(J2534_ULONG target, J2534_ULONG id, void *input, void *output) {
    J2534_LONG status = checkDevice();
    if (status) {
        return status;
    }

    if (id == GET_DEVICE_INFO || id == GET_PROTOCOL_INFO) {
        return discoveryIoctl(target, id, input, output);
    }

    // Voltage and discovery requests use a device handle.
    if (id == READ_PIN_VOLTAGE || id == READ_PROG_VOLTAGE) {
        return readVoltageIoctl(target, id, input, output);
    }

    Channel *channel = NULL;
    status = checkChannel(target, channel);
    if (status) {
        return status;
    }

    switch (id) {
    case START_REPEAT_MESSAGE:
    case QUERY_REPEAT_MESSAGE:
    case STOP_REPEAT_MESSAGE:
        return repeatIoctl(*channel, id, input, output);
    case GET_CONFIG:
    case SET_CONFIG:
        return configureChannelIoctl(*channel, id, input, output);
    case FIVE_BAUD_INIT:
        return fiveBaudInitIoctl(*channel, input, output);
    case FAST_INIT:
        return fastInitIoctl(*channel, input, output);
    case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
    case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
        return updateLookupTableIoctl(*channel, id, input, output);
    default:
        return clearChannelIoctl(*channel, id, input, output);
    }
}

} // namespace host
