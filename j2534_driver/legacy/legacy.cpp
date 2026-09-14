// SPDX-License-Identifier: GPL-3.0-only
#include "legacy.h"
#include "backend.h"
#include <algorithm>
#include <deque>
#include <map>
#include <stdio.h>
#include <vector>

namespace legacy {

static const J2534_ULONG MAX_RX_BYTES = 65536;
static const J2534_ULONG MAX_RX_MESSAGES = 256;
static const J2534_ULONG MAX_TX_MESSAGES = 256;
static const J2534_ULONG ADDRESS_FLAGS = CAN_29BIT_ID | ISO15765_ADDR_TYPE;

struct Filter {
    J2534_ULONG type;
    J2534_ULONG nativeId;
    J2534_ULONG endpoint;
    PASSTHRU_MSG pattern;
    PASSTHRU_MSG flow;
};

struct Transmission {
    PASSTHRU_MSG message;
    J2534_ULONG endpoint;
    bool waiting;
    bool complete;
    bool failed;
    bool periodic;
    J2534_ULONG nativePeriodicId;

    Transmission()
        : endpoint(0),
          waiting(false),
          complete(false),
          failed(false),
          periodic(false),
          nativePeriodicId(0) {
        memset(&message, 0, sizeof(message));
    }
};

struct Channel {
    J2534_ULONG physical;
    J2534_ULONG protocol;
    J2534_ULONG flags;
    J2534_ULONG baudrate;
    bool loopback;
    bool overflow;
    J2534_ULONG receiveError;
    J2534_ULONG receiveBytes;
    J2534_ULONG lastReadEndpoint;
    std::deque<PASSTHRU_MSG> received;
    std::map<J2534_ULONG, Filter> filters;
    std::map<J2534_ULONG, Transmission> transmissions;
    std::map<J2534_ULONG, J2534_ULONG> isoConfig;

    Channel()
        : physical(0),
          protocol(0),
          flags(0),
          baudrate(0),
          loopback(false),
          overflow(false),
          receiveError(0),
          receiveBytes(0),
          lastReadEndpoint(0) {
        isoConfig[ISO15765_BS] = 0;
        isoConfig[ISO15765_STMIN] = 0;
        isoConfig[ISO15765_WFT_MAX] = 0;
    }
};

static Backend backend;
static J2534_ULONG deviceId;
static J2534_ULONG nativeDeviceId;
static bool disconnected;
static J2534_ULONG nextId = 0x400000;
static std::map<J2534_ULONG, Channel> channels;
static std::atomic<bool> busy(false);
static char lastError[80] = "No error";
static char errorDetail[80];

bool enter() {
    if (busy.exchange(true)) {
        return false;
    }
    errorDetail[0] = 0;
    return true;
}

void leave() {
    busy.store(false);
}

J2534_LONG finish(J2534_LONG status, const char *detail) {
    if (detail) {
        platform::copyText(errorDetail, sizeof(errorDetail), detail);
    }
    if (!status) {
        platform::copyText(lastError, sizeof(lastError), "No error");
    } else if (errorDetail[0]) {
        platform::format(lastError,
                         sizeof(lastError),
                         "0x%02" J2534_PRIX ": %s",
                         status,
                         errorDetail);
    } else {
        platform::format(lastError,
                         sizeof(lastError),
                         "J2534 04.04 error 0x%02" J2534_PRIX,
                         status);
    }
    return status;
}

static J2534_ULONG allocateId() {
    ++nextId;
    if (!nextId) {
        ++nextId;
    }
    return nextId;
}

static J2534_LONG translate(J2534_LONG status) {
    if (!status) {
        return STATUS_NOERROR;
    }
    backend.getLastError(errorDetail);
    if (status == ERR_DEVICE_NOT_CONNECTED) {
        disconnected = true;
    }
    switch (status) {
    case ERR_DEVICE_NOT_OPEN:
        return ERR_INVALID_DEVICE_ID;
    case ERR_NULL_REQUIRED:
    case ERR_IOCTL_PARAM_ID_NOT_SUPPORTED:
        return ERR_INVALID_IOCTL_VALUE;
    case ERR_FILTER_TYPE_NOT_SUPPORTED:
        return ERR_INVALID_FILTER_ID;
    case ERR_VOLTAGE_IN_USE:
    case ERR_PIN_IN_USE:
        return ERR_PIN_INVALID;
    case ERR_INIT_FAILED:
        return ERR_FAILED;
    case ERR_OPEN_FAILED:
        return ERR_DEVICE_NOT_CONNECTED;
    case ERR_BUFFER_TOO_SMALL:
    case ERR_INVALID_CHANNEL_DESCRIPTOR:
        return ERR_INVALID_MSG;
    case ERR_LOG_CHAN_NOT_ALLOWED:
    case ERR_SELECT_TYPE_NOT_SUPPORTED:
        return ERR_NOT_SUPPORTED;
    case ERR_CONCURRENT_API_CALL:
        return ERR_FAILED;
    default:
        if (static_cast<J2534_ULONG>(status) <= ERR_INVALID_DEVICE_ID) {
            return status;
        }
        return ERR_FAILED;
    }
}

static J2534_LONG findChannel(J2534_ULONG id, Channel *&channel) {
    if (!deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    if (disconnected) {
        return ERR_DEVICE_NOT_CONNECTED;
    }
    std::map<J2534_ULONG, Channel>::iterator found = channels.find(id);
    if (found == channels.end()) {
        return ERR_INVALID_CHANNEL_ID;
    }
    channel = &found->second;
    native::PASSTHRU_MSG unused = {};
    J2534_ULONG count = 0;
    return translate(backend.readMsgs(channel->physical, &unused, &count, 0));
}

static unsigned addressSize(const PASSTHRU_MSG &message) {
    if (message.TxFlags & ISO15765_ADDR_TYPE) {
        return 5;
    }
    return 4;
}

static void encodeMessage(const PASSTHRU_MSG &message,
                          J2534_ULONG protocol,
                          J2534_ULONG handle,
                          native::PASSTHRU_MSG &output) {
    memset(&output, 0, sizeof(output));
    output.ProtocolID = protocol;
    output.MsgHandle = handle;
    output.TxFlags = message.TxFlags;
    output.DataLength = message.DataSize;
    output.DataBuffer = const_cast<unsigned char *>(message.Data);
    output.DataBufferSize = sizeof(message.Data);
}

static void enqueue(Channel &channel, const PASSTHRU_MSG &message) {
    J2534_ULONG bytes = message.DataSize + 24;
    if (channel.received.size() >= MAX_RX_MESSAGES || channel.receiveBytes + bytes > MAX_RX_BYTES) {
        channel.overflow = true;
        return;
    }
    channel.received.push_back(message);
    channel.receiveBytes += bytes;
}

static void transmitIndications(Channel &channel,
                                Transmission &transmission,
                                J2534_ULONG timestamp) {
    const PASSTHRU_MSG &original = transmission.message;
    if (channel.protocol == ISO15765) {
        PASSTHRU_MSG indication = {};
        indication.ProtocolID = ISO15765;
        indication.RxStatus = TX_MSG_TYPE | TX_INDICATION | (original.TxFlags & ADDRESS_FLAGS);
        indication.Timestamp = timestamp;
        indication.DataSize = addressSize(original);
        memcpy(indication.Data, original.Data, indication.DataSize);
        enqueue(channel, indication);
    }
    if (channel.loopback) {
        PASSTHRU_MSG echo = original;
        echo.RxStatus = TX_MSG_TYPE | (original.TxFlags & ADDRESS_FLAGS);
        echo.Timestamp = timestamp;
        echo.ExtraDataIndex = echo.DataSize;
        enqueue(channel, echo);
    }
}

static void receiveNative(Channel &channel, const native::PASSTHRU_MSG &message) {
    if (message.RxStatus & BUFFER_OVERFLOW) {
        channel.overflow = true;
    }
    if (message.RxStatus & (TX_INDICATION_SUCCESS | TX_FAILED)) {
        std::map<J2534_ULONG, Transmission>::iterator found =
            channel.transmissions.find(message.MsgHandle);
        if (found == channel.transmissions.end()) {
            return;
        }
        Transmission &transmission = found->second;
        transmission.complete = true;
        transmission.failed = (message.RxStatus & TX_FAILED) != 0;
        if (!transmission.failed) {
            // 04.04 exposes ISO-TP TxDone before optional loopback, with the original CAN address.
            transmitIndications(channel, transmission, message.Timestamp);
        } else if (!transmission.waiting) {
            channel.receiveError = ERR_FAILED;
        }
        if (!transmission.waiting && !transmission.periodic) {
            channel.transmissions.erase(found);
        }
        return;
    }
    if (message.RxStatus & RX_ERROR) {
        channel.receiveError = ERR_FAILED;
        return;
    }
    if (message.RxStatus & TX_MSG_TYPE) {
        return;
    }
    if (channel.protocol == ISO15765 && message.ProtocolID != ISO15765_LOGICAL) {
        return;
    }

    PASSTHRU_MSG received = {};
    received.ProtocolID = channel.protocol;
    received.RxStatus = message.RxStatus & 0x19f;
    received.Timestamp = message.Timestamp;
    received.DataSize = message.DataLength;
    received.ExtraDataIndex = message.ExtraDataIndex;
    if (received.RxStatus & (START_OF_MESSAGE | RX_BREAK)) {
        received.ExtraDataIndex = 0;
    }
    memcpy(received.Data, message.DataBuffer, received.DataSize);
    enqueue(channel, received);
}

static std::vector<J2534_ULONG> endpoints(const Channel &channel) {
    std::vector<J2534_ULONG> result;
    result.push_back(channel.physical);
    for (std::map<J2534_ULONG, Filter>::const_iterator entry = channel.filters.begin();
         entry != channel.filters.end();
         ++entry) {
        if (entry->second.type == FLOW_CONTROL_FILTER) {
            result.push_back(entry->second.endpoint);
        }
    }
    return result;
}

static J2534_LONG readNative(Channel &channel, J2534_ULONG endpoint, bool &received) {
    received = false;
    unsigned char data[4128];
    native::PASSTHRU_MSG message = {};
    message.DataBuffer = data;
    message.DataBufferSize = sizeof(data);
    J2534_ULONG count = 1;
    J2534_LONG status = backend.readMsgs(endpoint, &message, &count, 0);
    if (status == ERR_BUFFER_EMPTY) {
        return STATUS_NOERROR;
    }
    if (status == ERR_BUFFER_OVERFLOW) {
        channel.overflow = true;
    } else if (status) {
        return translate(status);
    }
    if (count) {
        receiveNative(channel, message);
        channel.lastReadEndpoint = endpoint;
        received = true;
    }
    return STATUS_NOERROR;
}

static J2534_LONG pump(Channel &channel,
                       J2534_ULONG wantedMessages = 0,
                       uint32_t start = 0,
                       J2534_ULONG timeout = 0) {
    std::vector<J2534_ULONG> ready = endpoints(channel);
    if (ready.size() > 1) {
        native::SCHANNELSET set = {};
        set.ChannelCount = (J2534_ULONG)ready.size();
        set.ChannelList = &ready[0];
        J2534_LONG status = backend.select(&set, READABLE_TYPE, 0);
        if (status == ERR_BUFFER_EMPTY) {
            return STATUS_NOERROR;
        }
        if (status) {
            return translate(status);
        }
        ready.resize(set.ChannelCount);
    }

    // Resume after the last serviced endpoint when several ISO-TP peers are readable.
    std::vector<J2534_ULONG>::iterator previous =
        std::find(ready.begin(), ready.end(), channel.lastReadEndpoint);
    if (previous != ready.end()) {
        std::rotate(ready.begin(), previous + 1, ready.end());
    }
    for (size_t index = 0; index < ready.size(); ++index) {
        for (unsigned drained = 0; drained < 32; ++drained) {
            if (timeout && platform::milliseconds() - start >= timeout) {
                return STATUS_NOERROR;
            }
            bool received = false;
            J2534_LONG status = readNative(channel, ready[index], received);
            if (status) {
                return status;
            }
            if (wantedMessages && (channel.received.size() >= wantedMessages ||
                                   channel.overflow || channel.receiveError)) {
                return STATUS_NOERROR;
            }
            if (!wantedMessages && channel.transmissions.size() < MAX_TX_MESSAGES) {
                return STATUS_NOERROR;
            }
            if (!received) {
                break;
            }
        }
    }
    return STATUS_NOERROR;
}

static J2534_LONG checkTransmissionCapacity(Channel &channel) {
    if (channel.transmissions.size() < MAX_TX_MESSAGES) {
        return STATUS_NOERROR;
    }
    J2534_LONG status = pump(channel);
    if (status) {
        return status;
    }
    return channel.transmissions.size() < MAX_TX_MESSAGES ? STATUS_NOERROR : ERR_BUFFER_FULL;
}

J2534_LONG PassThruOpen(void *, J2534_ULONG *output) {
    if (!output) {
        return ERR_NULL_PARAMETER;
    }
    if (deviceId) {
        if (disconnected) {
            return ERR_DEVICE_NOT_CONNECTED;
        }
        return ERR_DEVICE_IN_USE;
    }
    backend.load(legacyAnchor);
    J2534_ULONG opened = 0;
    J2534_LONG status = translate(backend.open("J2534-1:", &opened));
    if (status) {
        backend.unload();
        return status;
    }
    disconnected = false;
    nativeDeviceId = opened;
    deviceId = allocateId();
    *output = deviceId;
    return STATUS_NOERROR;
}

J2534_LONG PassThruClose(J2534_ULONG id) {
    if (!deviceId || id != deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    J2534_LONG status = translate(backend.close(nativeDeviceId));
    if (status && status != ERR_DEVICE_NOT_CONNECTED) {
        return status;
    }
    channels.clear();
    deviceId = 0;
    nativeDeviceId = 0;
    disconnected = false;
    backend.unload();
    return status;
}

J2534_LONG PassThruConnect(J2534_ULONG id,
                           J2534_ULONG protocol,
                           J2534_ULONG flags,
                           J2534_ULONG baudrate,
                           J2534_ULONG *output) {
    if (!deviceId || id != deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    if (!output) {
        return ERR_NULL_PARAMETER;
    }
    J2534_ULONG nativeProtocol = protocol;
    J2534_ULONG pins[2] = {};
    J2534_ULONG pinCount = 1;
    switch (protocol) {
    case CAN:
    case ISO15765:
        nativeProtocol = CAN;
        pins[0] = 6;
        pins[1] = 14;
        pinCount = 2;
        break;
    case J1850PWM:
        pins[0] = 2;
        pins[1] = 10;
        pinCount = 2;
        break;
    case J1850VPW:
        pins[0] = 2;
        break;
    case ISO9141:
    case ISO14230:
        pins[0] = 7;
        if (!(flags & ISO9141_K_LINE_ONLY)) {
            pins[1] = 15;
            pinCount = 2;
        }
        break;
    default:
        return ERR_INVALID_PROTOCOL_ID;
    }

    native::RESOURCE_STRUCT resources = {1, pinCount, pins};
    J2534_ULONG physical = 0;
    J2534_LONG result =
        backend.connect(nativeDeviceId, nativeProtocol, flags, baudrate, resources, &physical);
    if (result == ERR_RESOURCE_CONFLICT) {
        return ERR_INVALID_PROTOCOL_ID;
    }
    if (result) {
        return translate(result);
    }

    Channel channel;
    channel.physical = physical;
    channel.protocol = protocol;
    channel.flags = flags;
    channel.baudrate = baudrate;
    J2534_ULONG handle = allocateId();
    channels[handle] = channel;
    *output = handle;
    return STATUS_NOERROR;
}

J2534_LONG PassThruDisconnect(J2534_ULONG id) {
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    status = translate(backend.disconnect(channel->physical));
    if (!status) {
        channels.erase(id);
    }
    return status;
}

static J2534_LONG validateMessage(const Channel &channel, const PASSTHRU_MSG &message) {
    if (message.ProtocolID != channel.protocol) {
        return ERR_MSG_PROTOCOL_ID;
    }
    if (!message.DataSize || message.DataSize > sizeof(message.Data)) {
        return ERR_INVALID_MSG;
    }
    if (channel.protocol == CAN || channel.protocol == ISO15765) {
        if (message.DataSize < 4) {
            return ERR_INVALID_MSG;
        }
        J2534_ULONG allowed = CAN_29BIT_ID;
        if (channel.protocol == ISO15765) {
            allowed |= ISO15765_ADDR_TYPE | ISO15765_FRAME_PAD;
            if (message.DataSize <= addressSize(message) ||
                message.DataSize > addressSize(message) + 4095) {
                return ERR_INVALID_MSG;
            }
        } else if (message.DataSize > 12) {
            return ERR_INVALID_MSG;
        }
        if (message.TxFlags & ~allowed) {
            return ERR_INVALID_FLAGS;
        }
        if (!(channel.flags & CAN_ID_BOTH) && ((message.TxFlags ^ channel.flags) & CAN_29BIT_ID)) {
            return ERR_INVALID_MSG;
        }
    }
    return STATUS_NOERROR;
}

static J2534_LONG routeMessage(Channel &channel,
                               const PASSTHRU_MSG &message,
                               J2534_ULONG handle,
                               native::PASSTHRU_MSG &nativeMessage,
                               unsigned char *singleFrame,
                               J2534_ULONG &endpoint) {
    J2534_LONG status = validateMessage(channel, message);
    if (status) {
        return status;
    }
    endpoint = channel.physical;
    encodeMessage(message, channel.protocol, handle, nativeMessage);
    if (channel.protocol != ISO15765) {
        return STATUS_NOERROR;
    }

    for (std::map<J2534_ULONG, Filter>::const_iterator entry = channel.filters.begin();
         entry != channel.filters.end();
         ++entry) {
        const Filter &filter = entry->second;
        if (filter.type == FLOW_CONTROL_FILTER &&
            (filter.flow.TxFlags & ADDRESS_FLAGS) == (message.TxFlags & ADDRESS_FLAGS) &&
            !memcmp(filter.flow.Data, message.Data, addressSize(message))) {
            endpoint = filter.endpoint;
            nativeMessage.ProtocolID = ISO15765_LOGICAL;
            return STATUS_NOERROR;
        }
    }

    unsigned headerBytes = addressSize(message);
    unsigned payloadBytes = message.DataSize - headerBytes;
    unsigned maximumPayload = 11 - headerBytes;
    if (payloadBytes > maximumPayload) {
        return ERR_NO_FLOW_CONTROL;
    }

    // 04.04 permits an unfiltered single-frame transmission, including functional requests.
    memset(singleFrame, 0, 12);
    memcpy(singleFrame, message.Data, headerBytes);
    singleFrame[headerBytes] = (unsigned char)payloadBytes;
    memcpy(singleFrame + headerBytes + 1, message.Data + headerBytes, payloadBytes);
    nativeMessage.ProtocolID = CAN;
    nativeMessage.TxFlags = message.TxFlags & CAN_29BIT_ID;
    nativeMessage.DataBuffer = singleFrame;
    nativeMessage.DataBufferSize = 12;
    nativeMessage.DataLength = headerBytes + 1 + payloadBytes;
    if (message.TxFlags & ISO15765_FRAME_PAD) {
        nativeMessage.DataLength = 12;
    }
    return STATUS_NOERROR;
}

J2534_LONG PassThruWriteMsgs(J2534_ULONG id,
                             PASSTHRU_MSG *messages,
                             J2534_ULONG *count,
                             J2534_ULONG timeout) {
    if (!count) {
        return ERR_NULL_PARAMETER;
    }
    J2534_ULONG requested = *count;
    *count = 0;
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    if (!messages) {
        return ERR_NULL_PARAMETER;
    }
    if (requested > 0xffffffffUL / sizeof(PASSTHRU_MSG)) {
        return ERR_EXCEEDED_LIMIT;
    }
    uint32_t start = platform::milliseconds();
    for (J2534_ULONG index = 0; index < requested; ++index) {
        J2534_ULONG token = allocateId();
        native::PASSTHRU_MSG encoded;
        unsigned char singleFrame[12];
        J2534_ULONG endpoint = 0;
        status = routeMessage(*channel, messages[index], token, encoded, singleFrame, endpoint);
        if (status) {
            return status;
        }

        status = checkTransmissionCapacity(*channel);
        if (status) {
            return status;
        }
        Transmission transmission;
        transmission.message = messages[index];
        transmission.endpoint = endpoint;
        transmission.waiting = timeout != 0;
        channel->transmissions[token] = transmission;
        for (;;) {
            J2534_ULONG queued = 1;
            status = backend.queueMsgs(endpoint, &encoded, &queued);
            if (status != ERR_BUFFER_FULL || !timeout) {
                break;
            }
            if (platform::milliseconds() - start >= timeout) {
                channel->transmissions.erase(token);
                return ERR_TIMEOUT;
            }
            status = pump(*channel, 0, start, timeout);
            if (status) {
                channel->transmissions.erase(token);
                return status;
            }
            platform::sleepMs(1);
        }
        if (status) {
            channel->transmissions.erase(token);
            return translate(status);
        }
        if (!timeout) {
            *count += 1;
            continue;
        }

        // Count a blocking write only after firmware reports confirmed transmission.
        for (;;) {
            bool received = false;
            status = readNative(*channel, endpoint, received);
            Transmission &pending = channel->transmissions[token];
            if (status) {
                pending.waiting = false;
                return status;
            }
            if (pending.complete) {
                bool failed = pending.failed;
                channel->transmissions.erase(token);
                if (failed) {
                    return ERR_FAILED;
                }
                *count += 1;
                break;
            }
            if (platform::milliseconds() - start >= timeout) {
                pending.waiting = false;
                return ERR_TIMEOUT;
            }
            if (!received) {
                platform::sleepMs(1);
            }
        }
        if (*count < requested && platform::milliseconds() - start >= timeout) {
            return ERR_TIMEOUT;
        }
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
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    if (!messages) {
        return ERR_NULL_PARAMETER;
    }
    if (requested > 0xffffffffUL / sizeof(PASSTHRU_MSG)) {
        return ERR_EXCEEDED_LIMIT;
    }
    if (!requested) {
        return STATUS_NOERROR;
    }

    uint32_t start = platform::milliseconds();
    bool pumped = false;
    for (;;) {
        while (*count < requested && !channel->received.empty()) {
            messages[*count] = channel->received.front();
            channel->receiveBytes -= channel->received.front().DataSize + 24;
            channel->received.pop_front();
            *count += 1;
        }
        if (channel->overflow) {
            channel->overflow = false;
            return ERR_BUFFER_OVERFLOW;
        }
        if (channel->receiveError) {
            status = channel->receiveError;
            channel->receiveError = 0;
            return status;
        }
        if (*count == requested) {
            return STATUS_NOERROR;
        }
        if (pumped && (!timeout || platform::milliseconds() - start >= timeout)) {
            if (!*count) {
                return ERR_BUFFER_EMPTY;
            }
            if (timeout) {
                return ERR_TIMEOUT;
            }
            return STATUS_NOERROR;
        }
        status = pump(*channel, requested - *count, start, timeout);
        if (status) {
            return status;
        }
        pumped = true;
        if (timeout && channel->received.empty() && !channel->overflow && !channel->receiveError) {
            platform::sleepMs(1);
        }
    }
}

static J2534_LONG setNativeConfig(J2534_ULONG endpoint, J2534_ULONG parameter, J2534_ULONG value) {
    native::SCONFIG config = {parameter, value};
    native::SCONFIG_LIST list = {1, &config};
    return translate(backend.ioctl(endpoint, SET_CONFIG, &list, NULL));
}

J2534_LONG PassThruStartMsgFilter(J2534_ULONG id,
                                  J2534_ULONG type,
                                  PASSTHRU_MSG *mask,
                                  PASSTHRU_MSG *pattern,
                                  PASSTHRU_MSG *flow,
                                  J2534_ULONG *output) {
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    if (!mask || !pattern || !output) {
        return ERR_NULL_PARAMETER;
    }
    if (mask->ProtocolID != channel->protocol || pattern->ProtocolID != channel->protocol) {
        return ERR_MSG_PROTOCOL_ID;
    }
    if (!mask->DataSize || mask->DataSize > 12 || mask->DataSize != pattern->DataSize ||
        mask->TxFlags != pattern->TxFlags) {
        return ERR_INVALID_MSG;
    }

    Filter filter = {};
    filter.type = type;
    filter.pattern = *pattern;
    if (channel->protocol != ISO15765) {
        if (type != PASS_FILTER && type != BLOCK_FILTER) {
            return ERR_INVALID_FILTER_ID;
        }
        native::PASSTHRU_MSG nativeMask;
        native::PASSTHRU_MSG nativePattern;
        encodeMessage(*mask, channel->protocol, 0, nativeMask);
        encodeMessage(*pattern, channel->protocol, 0, nativePattern);
        if (channel->protocol == ISO9141 || channel->protocol == ISO14230) {
            nativeMask.TxFlags &= ~WAIT_P3_MIN_ONLY;
            nativePattern.TxFlags &= ~WAIT_P3_MIN_ONLY;
        }
        filter.endpoint = channel->physical;
        status = translate(backend.startMsgFilter(channel->physical,
                                                  type,
                                                  &nativeMask,
                                                  &nativePattern,
                                                  &filter.nativeId));
    } else {
        if (type != FLOW_CONTROL_FILTER) {
            return ERR_INVALID_FILTER_ID;
        }
        if (!flow) {
            return ERR_NULL_PARAMETER;
        }
        if (flow->ProtocolID != ISO15765) {
            return ERR_MSG_PROTOCOL_ID;
        }
        if (flow->DataSize != mask->DataSize || flow->TxFlags != mask->TxFlags ||
            mask->DataSize != addressSize(*mask)) {
            return ERR_INVALID_MSG;
        }
        if (mask->TxFlags & ~(ADDRESS_FLAGS | ISO15765_FRAME_PAD)) {
            return ERR_INVALID_FLAGS;
        }
        for (unsigned index = 0; index < mask->DataSize; ++index) {
            if (mask->Data[index] != 0xff) {
                return ERR_NOT_SUPPORTED;
            }
        }
        for (std::map<J2534_ULONG, Filter>::const_iterator entry = channel->filters.begin();
             entry != channel->filters.end();
             ++entry) {
            const Filter &existing = entry->second;
            if ((existing.flow.TxFlags & ADDRESS_FLAGS) != (flow->TxFlags & ADDRESS_FLAGS)) {
                continue;
            }
            unsigned length = addressSize(*flow);
            if (!memcmp(existing.pattern.Data, pattern->Data, length) ||
                !memcmp(existing.flow.Data, flow->Data, length) ||
                !memcmp(existing.pattern.Data, flow->Data, length) ||
                !memcmp(existing.flow.Data, pattern->Data, length)) {
                return ERR_NOT_UNIQUE;
            }
        }

        native::ISO15765_CHANNEL_DESCRIPTOR descriptor = {};
        descriptor.LocalTxFlags = pattern->TxFlags & ADDRESS_FLAGS;
        descriptor.RemoteTxFlags = flow->TxFlags;
        memcpy(descriptor.LocalAddress, pattern->Data, pattern->DataSize);
        memcpy(descriptor.RemoteAddress, flow->Data, flow->DataSize);
        status = translate(backend.logicalConnect(channel->physical,
                                                  ISO15765_LOGICAL,
                                                  0,
                                                  &descriptor,
                                                  &filter.endpoint));
        if (status) {
            return status;
        }
        for (std::map<J2534_ULONG, J2534_ULONG>::const_iterator config = channel->isoConfig.begin();
             config != channel->isoConfig.end();
             ++config) {
            status = setNativeConfig(filter.endpoint, config->first, config->second);
            if (status) {
                backend.logicalDisconnect(filter.endpoint);
                return status;
            }
        }
        filter.flow = *flow;
    }
    if (status) {
        return status;
    }
    J2534_ULONG handle = allocateId();
    channel->filters[handle] = filter;
    *output = handle;
    return STATUS_NOERROR;
}

J2534_LONG PassThruStopMsgFilter(J2534_ULONG id, J2534_ULONG filterId) {
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    std::map<J2534_ULONG, Filter>::iterator found = channel->filters.find(filterId);
    if (found == channel->filters.end()) {
        return ERR_INVALID_FILTER_ID;
    }
    Filter &filter = found->second;
    if (filter.type == FLOW_CONTROL_FILTER) {
        status = translate(backend.logicalDisconnect(filter.endpoint));
        if (!status) {
            std::map<J2534_ULONG, Transmission>::iterator transmission =
                channel->transmissions.begin();
            while (transmission != channel->transmissions.end()) {
                if (transmission->second.endpoint == filter.endpoint) {
                    std::map<J2534_ULONG, Transmission>::iterator removed = transmission;
                    ++transmission;
                    channel->transmissions.erase(removed);
                } else {
                    ++transmission;
                }
            }
        }
    } else {
        status = translate(backend.stopMsgFilter(channel->physical, filter.nativeId));
    }
    if (!status) {
        channel->filters.erase(found);
    }
    return status;
}

J2534_LONG PassThruStartPeriodicMsg(J2534_ULONG id,
                                    PASSTHRU_MSG *message,
                                    J2534_ULONG *output,
                                    J2534_ULONG interval) {
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    if (!message || !output) {
        return ERR_NULL_PARAMETER;
    }
    if (channel->protocol == ISO15765 && message->DataSize > 11) {
        return ERR_INVALID_MSG;
    }
    J2534_ULONG token = allocateId();
    J2534_ULONG endpoint = 0;
    native::PASSTHRU_MSG encoded;
    unsigned char singleFrame[12];
    status = routeMessage(*channel, *message, token, encoded, singleFrame, endpoint);
    if (status) {
        return status;
    }
    status = checkTransmissionCapacity(*channel);
    if (status) {
        return status;
    }
    Transmission transmission;
    transmission.message = *message;
    transmission.endpoint = endpoint;
    transmission.periodic = true;
    status = translate(
        backend.startPeriodicMsg(endpoint, &encoded, &transmission.nativePeriodicId, interval));
    if (status) {
        return status;
    }
    channel->transmissions[token] = transmission;
    *output = token;
    return STATUS_NOERROR;
}

J2534_LONG PassThruStopPeriodicMsg(J2534_ULONG id, J2534_ULONG messageId) {
    Channel *channel = NULL;
    J2534_LONG status = findChannel(id, channel);
    if (status) {
        return status;
    }
    std::map<J2534_ULONG, Transmission>::iterator found = channel->transmissions.find(messageId);
    if (found == channel->transmissions.end() || !found->second.periodic) {
        return ERR_INVALID_MSG_ID;
    }
    status =
        translate(backend.stopPeriodicMsg(found->second.endpoint, found->second.nativePeriodicId));
    if (!status) {
        channel->transmissions.erase(found);
    }
    return status;
}

J2534_LONG PassThruSetProgrammingVoltage(J2534_ULONG id, J2534_ULONG pin, J2534_ULONG voltage) {
    if (!deviceId || id != deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    native::RESOURCE_STRUCT resources = {1, 1, &pin};
    return translate(backend.setProgrammingVoltage(nativeDeviceId, resources, voltage));
}

J2534_LONG PassThruReadVersion(J2534_ULONG id, char *firmware, char *dll, char *api) {
    if (!firmware || !dll || !api) {
        return ERR_NULL_PARAMETER;
    }
    firmware[0] = 0;
    platform::copyText(dll, 80, "OpenDIAG 0.3.2");
    platform::copyText(api, 80, "04.04");
    if (!deviceId || id != deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    char backendDll[80];
    char backendApi[80];
    return translate(backend.readVersion(nativeDeviceId, firmware, backendDll, backendApi));
}

J2534_LONG PassThruGetLastError(char *description) {
    if (!description) {
        return ERR_NULL_PARAMETER;
    }
    platform::copyText(description, 80, lastError);
    return STATUS_NOERROR;
}

static J2534_LONG configurationIoctl(Channel &channel, J2534_ULONG id, void *input) {
    if (!input) {
        return ERR_NULL_PARAMETER;
    }
    SCONFIG_LIST &list = *(SCONFIG_LIST *)input;
    if (list.NumOfParams && !list.ConfigPtr) {
        return ERR_NULL_PARAMETER;
    }
    for (J2534_ULONG index = 0; index < list.NumOfParams; ++index) {
        SCONFIG &config = list.ConfigPtr[index];
        if (config.Parameter == LOOPBACK) {
            if (id == SET_CONFIG) {
                if (config.Value > 1) {
                    return ERR_INVALID_IOCTL_VALUE;
                }
                channel.loopback = config.Value != 0;
            } else {
                config.Value = channel.loopback ? 1 : 0;
            }
            continue;
        }
        if (config.Parameter == J1962_PINS) {
            J2534_ULONG pins = 0;
            if (channel.protocol == CAN || channel.protocol == ISO15765) {
                pins = 0x060e;
            } else if (channel.protocol == J1850PWM) {
                pins = 0x020a;
            } else if (channel.protocol == J1850VPW) {
                pins = 0x0200;
            } else {
                pins = (channel.flags & ISO9141_K_LINE_ONLY) ? 0x0700 : 0x070f;
            }
            if (id == SET_CONFIG && config.Value != pins) {
                return ERR_NOT_SUPPORTED;
            }
            config.Value = pins;
            continue;
        }
        if (channel.protocol == ISO15765) {
            std::map<J2534_ULONG, J2534_ULONG>::iterator cached =
                channel.isoConfig.find(config.Parameter);
            if (cached != channel.isoConfig.end()) {
                if (id == GET_CONFIG) {
                    config.Value = cached->second;
                    continue;
                }
                if (config.Value > 255 ||
                    (config.Parameter == ISO15765_STMIN && config.Value > 0x7f &&
                     (config.Value < 0xf1 || config.Value > 0xf9))) {
                    return ERR_INVALID_IOCTL_VALUE;
                }
                for (std::map<J2534_ULONG, Filter>::const_iterator filter = channel.filters.begin();
                     filter != channel.filters.end();
                     ++filter) {
                    J2534_LONG status =
                        setNativeConfig(filter->second.endpoint, config.Parameter, config.Value);
                    if (status) {
                        return status;
                    }
                }
                cached->second = config.Value;
                continue;
            }
            if (config.Parameter == BS_TX || config.Parameter == STMIN_TX) {
                if (id == SET_CONFIG && config.Value != 0xffff) {
                    return ERR_NOT_SUPPORTED;
                }
                config.Value = 0xffff;
                continue;
            }
            if (config.Parameter != DATA_RATE) {
                return ERR_INVALID_IOCTL_VALUE;
            }
        }
        native::SCONFIG translated = {config.Parameter, config.Value};
        native::SCONFIG_LIST nativeList = {1, &translated};
        J2534_LONG status = translate(backend.ioctl(channel.physical, id, &nativeList, NULL));
        if (status) {
            return status;
        }
        if (id == GET_CONFIG) {
            config.Value = translated.Value;
        }
    }
    return STATUS_NOERROR;
}

static J2534_LONG clearIoctl(J2534_ULONG id, Channel &channel, J2534_ULONG operation) {
    if (operation == CLEAR_MSG_FILTERS) {
        while (!channel.filters.empty()) {
            J2534_LONG status = PassThruStopMsgFilter(id, channel.filters.begin()->first);
            if (status) {
                return status;
            }
        }
        return STATUS_NOERROR;
    }
    std::vector<J2534_ULONG> targets = endpoints(channel);
    for (size_t index = 0; index < targets.size(); ++index) {
        J2534_LONG status = translate(backend.ioctl(targets[index], operation, NULL, NULL));
        if (status) {
            return status;
        }
    }
    if (operation == CLEAR_RX_BUFFER) {
        channel.received.clear();
        channel.receiveBytes = 0;
        channel.overflow = false;
        channel.receiveError = 0;
    }
    if (operation == CLEAR_TX_BUFFER || operation == CLEAR_PERIODIC_MSGS) {
        std::map<J2534_ULONG, Transmission>::iterator entry = channel.transmissions.begin();
        while (entry != channel.transmissions.end()) {
            bool periodic = entry->second.periodic;
            bool remove = (operation == CLEAR_PERIODIC_MSGS && periodic) ||
                          (operation == CLEAR_TX_BUFFER && !periodic);
            if (remove) {
                std::map<J2534_ULONG, Transmission>::iterator removed = entry;
                ++entry;
                channel.transmissions.erase(removed);
            } else {
                ++entry;
            }
        }
    }
    return STATUS_NOERROR;
}

J2534_LONG PassThruIoctl(J2534_ULONG target, J2534_ULONG id, void *input, void *output) {
    if (!deviceId) {
        return ERR_INVALID_DEVICE_ID;
    }
    if (id == READ_VBATT || id == READ_PROG_VOLTAGE) {
        if (target != deviceId) {
            return ERR_INVALID_DEVICE_ID;
        }
        if (!output) {
            return ERR_NULL_PARAMETER;
        }
        J2534_ULONG voltage = 0;
        J2534_ULONG pin = 16;
        native::RESOURCE_STRUCT resource = {1, 1, &pin};
        void *nativeInput = NULL;
        if (id == READ_VBATT) {
            nativeInput = &resource;
        }
        J2534_LONG status = translate(backend.ioctl(nativeDeviceId, id, nativeInput, &voltage));
        if (!status) {
            *(J2534_ULONG *)output = ((voltage + 50) / 100) * 100;
        }
        return status;
    }
    Channel *channel = NULL;
    J2534_LONG status = findChannel(target, channel);
    if (status) {
        return status;
    }
    switch (id) {
    case GET_CONFIG:
    case SET_CONFIG:
        return configurationIoctl(*channel, id, input);
    case CLEAR_TX_BUFFER:
    case CLEAR_RX_BUFFER:
    case CLEAR_PERIODIC_MSGS:
    case CLEAR_MSG_FILTERS:
        return clearIoctl(target, *channel, id);
    case FIVE_BAUD_INIT: {
        if (!input || !output) {
            return ERR_NULL_PARAMETER;
        }
        SBYTE_ARRAY &source = *(SBYTE_ARRAY *)input;
        SBYTE_ARRAY &destination = *(SBYTE_ARRAY *)output;
        native::SBYTE_ARRAY nativeSource = {source.NumOfBytes, source.BytePtr};
        native::SBYTE_ARRAY nativeDestination = {2, destination.BytePtr};
        status = translate(backend.ioctl(channel->physical, id, &nativeSource, &nativeDestination));
        if (!status) {
            destination.NumOfBytes = nativeDestination.NumOfBytes;
        }
        return status;
    }
    case FAST_INIT: {
        native::PASSTHRU_MSG nativeSource = {};
        native::PASSTHRU_MSG nativeDestination = {};
        unsigned char data[4128];
        if (input) {
            PASSTHRU_MSG &source = *(PASSTHRU_MSG *)input;
            if (source.DataSize > sizeof(source.Data)) {
                return ERR_INVALID_MSG;
            }
            encodeMessage(source, source.ProtocolID, 0, nativeSource);
        }
        nativeDestination.DataBuffer = data;
        nativeDestination.DataBufferSize = sizeof(data);
        status = translate(backend.ioctl(channel->physical,
                                         id,
                                         input ? &nativeSource : NULL,
                                         output ? &nativeDestination : NULL));
        if (!status && output) {
            PASSTHRU_MSG &destination = *(PASSTHRU_MSG *)output;
            memset(&destination, 0, sizeof(destination));
            destination.ProtocolID = nativeDestination.ProtocolID;
            destination.Timestamp = nativeDestination.Timestamp;
            destination.RxStatus = nativeDestination.RxStatus;
            destination.DataSize = nativeDestination.DataLength;
            destination.ExtraDataIndex = nativeDestination.ExtraDataIndex;
            memcpy(destination.Data, data, destination.DataSize);
        }
        return status;
    }
    case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
    case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE: {
        if (!input) {
            return ERR_NULL_PARAMETER;
        }
        SBYTE_ARRAY &source = *(SBYTE_ARRAY *)input;
        native::SBYTE_ARRAY bytes = {source.NumOfBytes, source.BytePtr};
        return translate(backend.ioctl(channel->physical, id, &bytes, NULL));
    }
    case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
        return translate(backend.ioctl(channel->physical, id, NULL, NULL));
    default:
        return ERR_INVALID_IOCTL_ID;
    }
}

} // namespace legacy
