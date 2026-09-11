// SPDX-License-Identifier: GPL-3.0-only
#include "../include/j2534.h"
#include "../platform/platform.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

struct Api {
    platform::Library library;
    decltype(&::PassThruOpen) open;
    decltype(&::PassThruClose) close;
    decltype(&::PassThruConnect) connect;
    decltype(&::PassThruDisconnect) disconnect;
    decltype(&::PassThruLogicalConnect) logicalConnect;
    decltype(&::PassThruLogicalDisconnect) logicalDisconnect;
    decltype(&::PassThruReadMsgs) read;
    decltype(&::PassThruQueueMsgs) queue;
    decltype(&::PassThruSelect) select;
    decltype(&::PassThruStartPeriodicMsg) startPeriodic;
    decltype(&::PassThruStopPeriodicMsg) stopPeriodic;
    decltype(&::PassThruStartMsgFilter) startFilter;
    decltype(&::PassThruStopMsgFilter) stopFilter;
    decltype(&::PassThruIoctl) ioctl;
    decltype(&::PassThruReadVersion) version;
    decltype(&::PassThruGetLastError) error;
    decltype(&::PassThruScanForDevices) scan;
    decltype(&::PassThruGetNextDevice) next;
    decltype(&::PassThruSetProgrammingVoltage) voltage;

    explicit Api(const char *path) {
        library = platform::openLibrary(path);
        if (!library) {
            throw std::runtime_error("LoadLibrary failed");
        }
        load(open, "PassThruOpen", 1);
        load(close, "PassThruClose", 2);
        load(connect, "PassThruConnect", 3);
        load(disconnect, "PassThruDisconnect", 4);
        load(read, "PassThruReadMsgs", 5);
        load(startPeriodic, "PassThruStartPeriodicMsg", 7);
        load(stopPeriodic, "PassThruStopPeriodicMsg", 8);
        load(startFilter, "PassThruStartMsgFilter", 9);
        load(stopFilter, "PassThruStopMsgFilter", 10);
        load(voltage, "PassThruSetProgrammingVoltage", 11);
        load(version, "PassThruReadVersion", 12);
        load(error, "PassThruGetLastError", 13);
        load(ioctl, "PassThruIoctl", 14);
        load(queue, "PassThruQueueMsgs", 15);
        load(scan, "PassThruScanForDevices", 16);
        load(next, "PassThruGetNextDevice", 17);
        load(logicalConnect, "PassThruLogicalConnect", 18);
        load(logicalDisconnect, "PassThruLogicalDisconnect", 19);
        load(select, "PassThruSelect", 20);
    }

    template <typename T> void load(T &function, const char *name, unsigned) {
        void *address = platform::findSymbol(library, name);
        if (!address) {
            throw std::runtime_error(std::string("Export name/ordinal mismatch: ") + name);
        }
        function = reinterpret_cast<T>(address);
    }

    ~Api() {
        platform::closeLibrary(library);
    }

    void require(J2534_LONG result, J2534_LONG expected, const char *operation) {
        if (result != expected) {
            char text[80] = {};
            error(text);
            printf("FAIL %s: got 0x%02" J2534_PRIX ", expected 0x%02" J2534_PRIX " (%s)\n",
                   operation,
                   result,
                   expected,
                   text);
            throw std::runtime_error(operation);
        }
    }
};

static void check(bool condition, const char *description) {
    if (!condition) {
        throw std::runtime_error(description);
    }
}

struct Session {
    Api &api;
    J2534_ULONG id;
    Session(Api &functions, const char *name)
        : api(functions),
          id(0) {
        api.require(api.open(name, &id), STATUS_NOERROR, "Open");
    }

  private:
    Session(const Session &);
    Session &operator=(const Session &);

  public:
    ~Session() {
        if (id) {
            api.close(id);
        }
    }
};

struct Message {
    PASSTHRU_MSG value;
    unsigned char data[4128];
    Message() {
        memset(&value, 0, sizeof(value));
        memset(data, 0, sizeof(data));
        value.DataBuffer = data;
        value.DataBufferSize = sizeof(data);
    }
};

static J2534_ULONG connectCan(Api &api, J2534_ULONG device) {
    J2534_ULONG pins[] = {6, 14};
    RESOURCE_STRUCT resources = {J1962_CONNECTOR, 2, pins};
    J2534_ULONG channel = 0;
    api.require(api.connect(device, CAN, CAN_29BIT_ID, 500000, resources, &channel),
                STATUS_NOERROR,
                "Connect CAN");
    return channel;
}

static void putId(unsigned char *bytes, J2534_ULONG id) {
    for (unsigned i = 0; i < 4; ++i) {
        bytes[i] = (unsigned char)(id >> (24 - 8 * i));
    }
}

static J2534_ULONG connectIso(Api &api, J2534_ULONG physical) {
    ISO15765_CHANNEL_DESCRIPTOR descriptor = {};
    descriptor.LocalTxFlags = CAN_29BIT_ID;
    descriptor.RemoteTxFlags = CAN_29BIT_ID | ISO15765_FRAME_PAD;
    putId(descriptor.LocalAddress, 0x18daf111);
    putId(descriptor.RemoteAddress, 0x18da11f1);
    J2534_ULONG channel = 0;
    api.require(api.logicalConnect(physical, ISO15765_LOGICAL, 0, &descriptor, &channel),
                STATUS_NOERROR,
                "LogicalConnect");
    return channel;
}

static void selfTest(Api &api, const char *name) {
    static_assert(sizeof(J2534_ULONG) == 4, "32-bit J2534_ULONG required");
    static_assert(sizeof(PASSTHRU_MSG) == 32 + sizeof(void *), "05.00 message ABI");
    static_assert(sizeof(RESOURCE_STRUCT) == 8 + sizeof(void *), "resource ABI");
    static_assert(sizeof(ISO15765_CHANNEL_DESCRIPTOR) == 18, "descriptor packing");
    static_assert(sizeof(SDEVICE) == 104, "device ABI");
    // Validate discovery and pre-open errors before creating a session.
    J2534_ULONG unchanged = 0x12345678;
    api.require(api.open(NULL, &unchanged), ERR_NULL_PARAMETER, "Open NULL");
    check(unchanged == 0x12345678, "Open failure modified output");
    api.require(api.open("J2534-1:NotOpenDIAG", &unchanged), ERR_OPEN_FAILED, "Open invalid name");
    api.require(api.close(0), ERR_DEVICE_NOT_OPEN, "Close before Open");
    api.require(api.next(NULL), ERR_NULL_PARAMETER, "GetNextDevice NULL");
    J2534_ULONG found = 0;
    api.require(api.scan(&found), STATUS_NOERROR, "Scan");
    check(found == 1, "Configured adapter not discovered");
    SDEVICE info;
    api.require(api.next(&info), STATUS_NOERROR, "GetNextDevice");
    check(info.DeviceDLLFWStatus == DEVICE_DLL_FW_COMPATIBLE, "Firmware not compatible");
    api.require(api.next(&info), ERR_EXCEEDED_LIMIT, "Discovery exhaustion");
    Session session(api, name);
    api.require(api.open(name, &unchanged), ERR_DEVICE_IN_USE, "Duplicate Open");
    check(unchanged == 0x12345678, "Duplicate Open modified output");
    char firmware[80];
    char dll[80];
    char version[80];
    api.require(api.version(session.id, firmware, dll, version), STATUS_NOERROR, "ReadVersion");
    check(strcmp(version, "05.00") == 0, "API version is not 05.00");
    printf("Firmware: %s\nDLL: %s; API: %s\n", firmware, dll, version);
    api.require(api.version(0, firmware, dll, version),
                ERR_INVALID_DEVICE_ID,
                "Invalid device handle");
    api.require(api.version(0xffffffffUL, firmware, dll, version),
                ERR_INVALID_DEVICE_ID,
                "Invalid maximum device handle");
    // Read supply voltage and verify that the battery pin cannot be driven.
    J2534_ULONG pin = 16;
    RESOURCE_STRUCT resource = {J1962_CONNECTOR, 1, &pin};
    J2534_ULONG millivolts = 0;
    api.require(api.ioctl(session.id, READ_PIN_VOLTAGE, &resource, &millivolts),
                STATUS_NOERROR,
                "Battery voltage");
    printf("Battery: %" J2534_PRIu " mV\n", millivolts);
    api.require(api.voltage(session.id, resource, 12000),
                ERR_PIN_NOT_SUPPORTED,
                "Reject voltage on battery pin");
    api.require(api.ioctl(session.id, READ_PROG_VOLTAGE, NULL, &millivolts),
                STATUS_NOERROR,
                "Programming voltage off");
    check(millivolts == 0, "Inactive programming voltage must report zero");
    J2534_ULONG physical = connectCan(api, session.id);
    // Empty reads and Select must honor the caller timeout.
    Message received;
    J2534_ULONG count = 0;
    api.require(api.read(physical, &received.value, &count, 100),
                STATUS_NOERROR,
                "Zero-count read");
    count = 1;
    api.require(api.read(physical, &received.value, &count, 0),
                ERR_BUFFER_EMPTY,
                "Empty nonblocking read");
    check(count == 0, "Empty count");
    uint32_t start = platform::milliseconds();
    count = 1;
    api.require(api.read(physical, &received.value, &count, 100),
                ERR_BUFFER_EMPTY,
                "Empty timed read");
    uint32_t elapsed = platform::milliseconds() - start;
    printf("Empty read timeout: %" J2534_PRIu " ms (requested 100)\n", elapsed);
    check(elapsed >= 100 && elapsed <= 150, "Read timeout tolerance");
    J2534_ULONG ids[] = {physical};
    SCHANNELSET set = {1, 1, ids};
    start = platform::milliseconds();
    api.require(api.select(&set, READABLE_TYPE, 100), ERR_BUFFER_EMPTY, "Empty Select");
    elapsed = platform::milliseconds() - start;
    printf("Empty Select timeout: %" J2534_PRIu " ms (requested 100)\n", elapsed);
    check(set.ChannelCount == 0 && elapsed >= 100 && elapsed <= 150, "Select timeout tolerance");
    // Configuration batches must stop at the first unsupported parameter.
    SCONFIG configs[40];
    for (unsigned i = 0; i < 40; ++i) {
        configs[i].Parameter = DATA_RATE;
        configs[i].Value = 0;
    }

    SCONFIG_LIST configList = {40, configs};
    api.require(api.ioctl(physical, GET_CONFIG, &configList, NULL),
                STATUS_NOERROR,
                "Batched GET_CONFIG");
    for (unsigned i = 0; i < 40; ++i) {
        check(configs[i].Value == 500000, "GET_CONFIG value");
    }

    configs[1].Parameter = 0xffff;
    configs[2].Value = 123;
    configList.NumOfParams = 3;
    api.require(api.ioctl(physical, GET_CONFIG, &configList, NULL),
                ERR_IOCTL_PARAM_ID_NOT_SUPPORTED,
                "Configuration stops at error");
    check(configs[2].Value == 123, "Unprocessed config modified");
    api.require(api.ioctl(physical, CLEAR_TX_QUEUE, &pin, NULL),
                ERR_NULL_REQUIRED,
                "Required NULL IOCTL");
    // Filters and logical channels must invalidate their handles when removed.
    Message mask;
    Message pattern;
    mask.value.ProtocolID = CAN;
    pattern.value.ProtocolID = CAN;
    mask.value.TxFlags = CAN_29BIT_ID;
    pattern.value.TxFlags = CAN_29BIT_ID;
    mask.value.DataLength = 4;
    pattern.value.DataLength = 4;
    memset(mask.data, 0xff, 4);
    putId(pattern.data, 0x1ffffffe);
    J2534_ULONG filter = 0;
    api.require(api.startFilter(physical, PASS_FILTER, &mask.value, &pattern.value, &filter),
                STATUS_NOERROR,
                "Start filter");
    api.require(api.stopFilter(physical, filter), STATUS_NOERROR, "Stop filter");
    api.require(api.stopFilter(physical, filter), ERR_INVALID_FILTER_ID, "Stale filter");
    J2534_ULONG logical = connectIso(api, physical);
    api.require(api.logicalDisconnect(logical), STATUS_NOERROR, "LogicalDisconnect");
    count = 0;
    api.require(api.read(logical, &received.value, &count, 0),
                ERR_INVALID_CHANNEL_ID,
                "Stale logical handle");
    logical = connectIso(api, physical);
    api.require(api.disconnect(physical), STATUS_NOERROR, "Parent disconnect");
    api.require(api.logicalDisconnect(logical),
                ERR_INVALID_CHANNEL_ID,
                "Child invalidated with parent");
    // Reopening must not make a stale device handle valid again.
    J2534_ULONG oldDevice = session.id;
    api.require(api.close(session.id), STATUS_NOERROR, "Close");
    session.id = 0;
    api.require(api.open(name, &session.id), STATUS_NOERROR, "Reopen");
    check(session.id != oldDevice, "Device handle reused");
    api.require(api.close(oldDevice), ERR_INVALID_DEVICE_ID, "Stale device ID after reopen");
    printf("PASS ABI, discovery, handles, pointers, IOCTLs, timeout and filter tests\n");
}

static void contractTest(Api &api, const char *name) {
    {
        Session session(api, name);
        J2534_ULONG physical = connectCan(api, session.id);
        J2534_ULONG logical = connectIso(api, physical);
        for (J2534_ULONG fault = 0xfe01; fault <= 0xfe04; ++fault) {
            SCONFIG config = {fault, 0};
            SCONFIG_LIST list = {1, &config};
            api.require(api.ioctl(physical, GET_CONFIG, &list, NULL),
                        STATUS_NOERROR,
                        "Configure Select response");
            J2534_ULONG ids[] = {physical, logical, 0x12345678};
            J2534_ULONG capacity = fault == 0xfe01 ? 1 : 2;
            SCHANNELSET set = {capacity, 0, ids};
            J2534_LONG status = api.select(&set, READABLE_TYPE, 0);
            if (fault == 0xfe04) {
                api.require(status, STATUS_NOERROR, "Reordered Select response");
                check(set.ChannelCount == 2 && ids[0] == logical && ids[1] == physical,
                      "Select response handle mapping");
            } else {
                check(ids[0] == physical && ids[1] == logical && ids[2] == 0x12345678 &&
                          set.ChannelCount == capacity,
                      "Malformed Select response modified caller storage");
                api.require(status, ERR_INVALID_MSG, "Malformed Select response");
            }
        }
        printf("PASS Select response bounds, uniqueness and caller storage\n");
    }
    {
        Session session(api, name);
        J2534_ULONG channel = connectCan(api, session.id);
        Message sent;
        sent.value.ProtocolID = CAN;
        sent.value.DataLength = sizeof(sent.data);
        sent.value.MsgHandle = 42;
        for (unsigned i = 0; i < sizeof(sent.data); ++i) {
            sent.data[i] = (unsigned char)(i * 37);
        }
        J2534_ULONG count = 1;
        api.require(api.queue(channel, &sent.value, &count), STATUS_NOERROR, "Fragmented Queue");
        Message received;
        count = 1;
        api.require(api.read(channel, &received.value, &count, 0),
                    STATUS_NOERROR,
                    "Fragmented Read");
        check(count == 1 && received.value.DataLength == sizeof(sent.data), "Large message size");
        check(received.value.DataBuffer == received.data &&
                  received.value.DataBufferSize == sizeof(received.data),
              "Caller buffer ownership changed");
        check(!memcmp(sent.data, received.data, sizeof(sent.data)), "Large message corruption");
        check(received.value.MsgHandle == 42 && received.value.Timestamp == 0xfffffffe &&
                  received.value.ExtraDataIndex == sizeof(sent.data),
              "Message metadata changed");

        sent.value.DataLength = 8;
        PASSTHRU_MSG batch[] = {sent.value, sent.value};
        batch[1].MsgHandle = 0xf001;
        count = 2;
        api.require(api.queue(channel, batch, &count), ERR_BUFFER_FULL, "Queue accepted prefix");
        check(count == 1, "Accepted prefix count");
        Message first;
        Message second;
        PASSTHRU_MSG outputs[] = {first.value, second.value};
        count = 2;
        uint32_t start = platform::milliseconds();
        api.require(api.read(channel, outputs, &count, 100), ERR_TIMEOUT, "Partial timed Read");
        uint32_t elapsed = platform::milliseconds() - start;
        check(count == 1 && elapsed >= 100 && elapsed <= 150, "Partial read count or timing");
        check(outputs[1].DataLength == 0, "Read modified unused output");

        sent.value.MsgHandle = 0xf002;
        count = 1;
        api.require(api.queue(channel, &sent.value, &count), STATUS_NOERROR, "Queue overflow");
        count = 1;
        api.require(api.read(channel, &received.value, &count, 0),
                    ERR_BUFFER_OVERFLOW,
                    "Read overflow indication");
        check(count == 1 && (received.value.RxStatus & BUFFER_OVERFLOW), "Overflow count");

        sent.value.MsgHandle = 43;
        count = 1;
        api.require(api.queue(channel, &sent.value, &count), STATUS_NOERROR, "Queue small buffer");
        received.value.DataBufferSize = 3;
        count = 1;
        api.require(api.read(channel, &received.value, &count, 0),
                    ERR_BUFFER_TOO_SMALL,
                    "Small receive buffer");
        check(count == 0 && received.value.DataLength == 3 &&
                  received.value.DataBuffer == received.data &&
                  received.value.DataBufferSize == 3 && !memcmp(received.data, sent.data, 3),
              "Truncated message contract");
        count = 1;
        api.require(api.read(channel, &received.value, &count, 0),
                    ERR_BUFFER_EMPTY,
                    "Small buffer suffix discarded");
    }
    // Each injected wire failure must latch disconnection until Close and Open.
    for (J2534_ULONG fault = 0xff01; fault <= 0xff07; ++fault) {
        Session session(api, name);
        J2534_ULONG channel = connectCan(api, session.id);
        SCONFIG config = {fault, 123};
        SCONFIG_LIST list = {1, &config};
        uint32_t start = platform::milliseconds();
        J2534_LONG result = api.ioctl(channel, GET_CONFIG, &list, NULL);
        if (fault == 0xff06) {
            api.require(result, STATUS_NOERROR, "Response before EOF");
            platform::sleepMs(20);
            result = api.ioctl(channel, GET_CONFIG, &list, NULL);
        }
        api.require(result, ERR_DEVICE_NOT_CONNECTED, "Wire fault");
        if (fault != 0xff06) {
            check(config.Value == 123, "Failed RPC modified config output");
        }
        if (fault == 0xff05) {
            uint32_t elapsed = platform::milliseconds() - start;
            check(elapsed >= 1000 && elapsed <= 1200, "RPC timeout bound");
        }
        Message message;
        J2534_ULONG count = 0;
        api.require(api.read(channel, &message.value, &count, 0),
                    ERR_DEVICE_NOT_CONNECTED,
                    "Disconnected state latched");
        api.require(api.close(session.id), ERR_DEVICE_NOT_CONNECTED, "Close failed connection");
        session.id = 0;
        printf("PASS wire fault 0x%04" J2534_PRIX ", disconnected latch and cleanup\n", fault);
    }

    Session reopened(api, name);
    connectCan(api, reopened.id);
    printf(
        "PASS fragmented messages, caller buffers, partial counts, overflow and fault recovery\n");
}

int main(int argc, char **argv) {
    try {
        if (argc != 3) {
            throw std::runtime_error("Expected library path and selftest/contract");
        }
        Api api(argv[1]);
        if (!strcmp(argv[2], "selftest")) {
            selfTest(api, "J2534-1:OpenDIAG");
        } else {
            contractTest(api, "J2534-1:OpenDIAG");
        }
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
