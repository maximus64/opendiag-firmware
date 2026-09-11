// SPDX-License-Identifier: GPL-3.0-only
#include "../include/j2534_0404.h"
#include "../platform/platform.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

static void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct Api {
    platform::Library library;
    decltype(&PassThruOpen) open;
    decltype(&PassThruClose) close;
    decltype(&PassThruConnect) connect;
    decltype(&PassThruDisconnect) disconnect;
    decltype(&PassThruReadMsgs) read;
    decltype(&PassThruWriteMsgs) write;
    decltype(&PassThruStartPeriodicMsg) startPeriodic;
    decltype(&PassThruStopPeriodicMsg) stopPeriodic;
    decltype(&PassThruStartMsgFilter) startFilter;
    decltype(&PassThruStopMsgFilter) stopFilter;
    decltype(&PassThruSetProgrammingVoltage) voltage;
    decltype(&PassThruReadVersion) version;
    decltype(&PassThruGetLastError) error;
    decltype(&PassThruIoctl) ioctl;

    explicit Api(const std::string &path) {
        library = platform::openLibrary(path.c_str());
        check(library != NULL, "Load 04.04 DLL");
        load(open, "PassThruOpen", 1);
        load(close, "PassThruClose", 2);
        load(connect, "PassThruConnect", 3);
        load(disconnect, "PassThruDisconnect", 4);
        load(read, "PassThruReadMsgs", 5);
        load(write, "PassThruWriteMsgs", 6);
        load(startPeriodic, "PassThruStartPeriodicMsg", 7);
        load(stopPeriodic, "PassThruStopPeriodicMsg", 8);
        load(startFilter, "PassThruStartMsgFilter", 9);
        load(stopFilter, "PassThruStopMsgFilter", 10);
        load(voltage, "PassThruSetProgrammingVoltage", 11);
        load(version, "PassThruReadVersion", 12);
        load(error, "PassThruGetLastError", 13);
        load(ioctl, "PassThruIoctl", 14);
        check(!platform::findSymbol(library, "PassThruQueueMsgs"), "05.00 export in legacy DLL");
    }

    ~Api() {
        platform::closeLibrary(library);
    }

    template <typename Function> void load(Function &function, const char *name, unsigned) {
        void *address = platform::findSymbol(library, name);
        check(address != NULL, name);
        function = reinterpret_cast<Function>(address);
    }

    void require(J2534_LONG actual, J2534_LONG expected, const char *operation) {
        if (actual != expected) {
            char detail[80] = {};
            error(detail);
            printf("FAIL %s: got 0x%02" J2534_PRIX ", expected 0x%02" J2534_PRIX " (%s)\n",
                   operation,
                   actual,
                   expected,
                   detail);
            throw std::runtime_error(operation);
        }
    }
};

struct Session {
    Api &api;
    J2534_ULONG id;

    explicit Session(Api &functions)
        : api(functions),
          id(0) {
        api.require(api.open(NULL, &id), STATUS_NOERROR, "Open NULL name");
    }

    ~Session() {
        if (id) {
            api.close(id);
        }
    }

  private:
    Session(const Session &);
    Session &operator=(const Session &);
};

static void putId(unsigned char *data, J2534_ULONG id) {
    for (unsigned index = 0; index < 4; ++index) {
        data[index] = (unsigned char)(id >> (24 - index * 8));
    }
}

static J2534_ULONG connect(Api &api, J2534_ULONG device, J2534_ULONG protocol) {
    J2534_ULONG channel = 0;
    api.require(api.connect(device, protocol, CAN_29BIT_ID, 500000, &channel),
                STATUS_NOERROR,
                "Connect");
    return channel;
}

static void configure(Api &api, J2534_ULONG channel, J2534_ULONG parameter, J2534_ULONG value) {
    SCONFIG config = {parameter, value};
    SCONFIG_LIST list = {1, &config};
    api.require(api.ioctl(channel, SET_CONFIG, &list, NULL), STATUS_NOERROR, "SET_CONFIG");
    config.Value = 0xdeadbeef;
    api.require(api.ioctl(channel, GET_CONFIG, &list, NULL), STATUS_NOERROR, "GET_CONFIG");
    check(config.Value == value, "Configuration round trip");
}

static J2534_ULONG flowFilter(Api &api, J2534_ULONG channel, J2534_ULONG protocol) {
    PASSTHRU_MSG mask = {};
    PASSTHRU_MSG pattern = {};
    PASSTHRU_MSG flow = {};
    mask.ProtocolID = protocol;
    pattern.ProtocolID = protocol;
    flow.ProtocolID = protocol;
    mask.TxFlags = CAN_29BIT_ID;
    pattern.TxFlags = CAN_29BIT_ID;
    flow.TxFlags = CAN_29BIT_ID;
    mask.DataSize = 4;
    pattern.DataSize = 4;
    flow.DataSize = 4;
    memset(mask.Data, 0xff, 4);
    putId(pattern.Data, 0x18daf111);
    putId(flow.Data, 0x18da11f1);
    J2534_ULONG filter = 0;
    J2534_ULONG type = protocol == ISO15765 ? FLOW_CONTROL_FILTER : PASS_FILTER;
    PASSTHRU_MSG *flowPointer = protocol == ISO15765 ? &flow : NULL;
    api.require(api.startFilter(channel, type, &mask, &pattern, flowPointer, &filter),
                STATUS_NOERROR,
                "Start filter");
    if (protocol == ISO15765) {
        J2534_ULONG unchanged = 0xdeadbeef;
        api.require(api.startFilter(channel, type, &mask, &pattern, &flow, &unchanged),
                    ERR_NOT_UNIQUE,
                    "Duplicate flow filter");
        check(unchanged == 0xdeadbeef, "Failed filter modified output");
    }
    return filter;
}

static void selfTest(Api &api) {
    static_assert(sizeof(PASSTHRU_MSG) == 4152, "04.04 inline message layout");
    api.require(api.open(NULL, NULL), ERR_NULL_PARAMETER, "NULL output");
    Session session(api);
    J2534_ULONG duplicate = 0x12345678;
    api.require(api.open(NULL, &duplicate), ERR_DEVICE_IN_USE, "Duplicate Open");
    check(duplicate == 0x12345678, "Open modified error output");
    char firmware[80];
    char dll[80];
    char version[80];
    api.require(api.version(session.id, firmware, dll, version), STATUS_NOERROR, "ReadVersion");
    check(!strcmp(version, "04.04"), "Wrong API version");
    printf("Firmware: %s\nDLL: %s; API: %s\n", firmware, dll, version);
    J2534_ULONG voltage = 0;
    api.require(api.ioctl(session.id, READ_VBATT, NULL, &voltage), STATUS_NOERROR, "READ_VBATT");
    check(voltage % 100 == 0, "04.04 voltage rounding");
    printf("Battery: %" J2534_PRIu " mV\n", voltage);
    api.require(api.voltage(session.id, 16, 12000), ERR_PIN_INVALID, "Battery pin drive rejected");

    J2534_ULONG channel = connect(api, session.id, ISO15765);
    configure(api, channel, LOOPBACK, 1);
    configure(api, channel, ISO15765_BS, 0);
    configure(api, channel, ISO15765_STMIN, 0);
    configure(api, channel, ISO15765_WFT_MAX, 0);
    configure(api, channel, BS_TX, 0xffff);
    configure(api, channel, J1962_PINS, 0x060e);
    PASSTHRU_MSG message = {};
    J2534_ULONG count = 0;
    api.require(api.read(channel, &message, &count, 100), STATUS_NOERROR, "Zero-count Read");
    count = 1;
    uint32_t start = platform::milliseconds();
    api.require(api.read(channel, &message, &count, 100), ERR_BUFFER_EMPTY, "Empty timed Read");
    uint32_t elapsed = platform::milliseconds() - start;
    check(!count && elapsed >= 100 && elapsed <= 150, "Read timeout contract");

    message.ProtocolID = ISO15765;
    message.TxFlags = CAN_29BIT_ID;
    message.DataSize = 12;
    putId(message.Data, 0x18da11f1);
    count = 1;
    api.require(api.write(channel, &message, &count, 1000),
                ERR_NO_FLOW_CONTROL,
                "Segmented write without filter");
    check(!count, "Failed write count");
    J2534_ULONG filter = flowFilter(api, channel, ISO15765);
    api.require(api.stopFilter(channel, filter), STATUS_NOERROR, "Stop flow filter");
    api.require(api.stopFilter(channel, filter), ERR_INVALID_FILTER_ID, "Stale flow filter");
    flowFilter(api, channel, ISO15765);
    api.require(api.ioctl(channel, CLEAR_MSG_FILTERS, NULL, NULL),
                STATUS_NOERROR,
                "Clear flow filters");
    api.require(api.disconnect(channel), STATUS_NOERROR, "Disconnect");
    count = 0;
    api.require(api.read(channel, &message, &count, 0), ERR_INVALID_CHANNEL_ID, "Stale channel");
    J2534_ULONG previous = session.id;
    api.require(api.close(session.id), STATUS_NOERROR, "Close");
    session.id = 0;
    api.require(api.open(NULL, &session.id), STATUS_NOERROR, "Reopen");
    api.require(api.close(previous), ERR_INVALID_DEVICE_ID, "Stale device");
    printf("PASS 04.04 ABI, exports, handles, configuration, filters and timeout\n");
}

static void contractTest(Api &api) {
    {
        Session session(api);
        J2534_ULONG channel = connect(api, session.id, ISO15765);
        flowFilter(api, channel, ISO15765);
        PASSTHRU_MSG sent = {};
        sent.ProtocolID = ISO15765;
        sent.TxFlags = CAN_29BIT_ID;
        sent.DataSize = 4099;
        putId(sent.Data, 0x18da11f1);
        for (unsigned index = 4; index < sent.DataSize; ++index) {
            sent.Data[index] = (unsigned char)(index * 13);
        }
        J2534_ULONG count = 1;
        api.require(api.write(channel, &sent, &count, 1000),
                    STATUS_NOERROR,
                    "Maximum segmented write");
        check(count == 1, "Maximum write count");
        PASSTHRU_MSG received[2] = {};
        count = 2;
        api.require(api.read(channel, received, &count, 0),
                    STATUS_NOERROR,
                    "Maximum segmented read");
        check(count == 2 && received[0].RxStatus == (CAN_29BIT_ID | TX_INDICATION | TX_MSG_TYPE),
              "TxDone conversion");
        check(received[1].DataSize == sent.DataSize &&
                  !memcmp(received[1].Data, sent.Data, sent.DataSize),
              "Fragmented payload corruption");
    }
    {
        Session session(api);
        J2534_ULONG channel = connect(api, session.id, CAN);
        PASSTHRU_MSG messages[3] = {};
        for (unsigned index = 0; index < 3; ++index) {
            messages[index].ProtocolID = CAN;
            messages[index].TxFlags = CAN_29BIT_ID;
            messages[index].DataSize = 5;
            putId(messages[index].Data, 0x18da11f1);
            messages[index].Data[4] = (unsigned char)(index + 1);
        }
        J2534_ULONG count = 3;
        api.require(api.write(channel, messages, &count, 1000),
                    STATUS_NOERROR,
                    "Blocking message batch");
        check(count == 3, "Blocking batch count");
        api.require(api.ioctl(channel, CLEAR_RX_BUFFER, NULL, NULL),
                    STATUS_NOERROR,
                    "Clear batch responses");

        messages[1].Data[4] = 0xe3;
        count = 3;
        api.require(api.write(channel, messages, &count, 0),
                    ERR_BUFFER_FULL,
                    "Partial nonblocking write");
        check(count == 1, "Accepted prefix count");
        api.require(api.ioctl(channel, CLEAR_RX_BUFFER, NULL, NULL),
                    STATUS_NOERROR,
                    "Clear prefix response");

        messages[0].Data[4] = 0xe1;
        count = 1;
        uint32_t start = platform::milliseconds();
        api.require(api.write(channel, messages, &count, 100),
                    ERR_TIMEOUT,
                    "Missing transmission confirmation");
        uint32_t elapsed = platform::milliseconds() - start;
        check(count == 0 && elapsed >= 100 && elapsed <= 150, "Write timeout contract");
        api.require(api.ioctl(channel, CLEAR_TX_BUFFER, NULL, NULL),
                    STATUS_NOERROR,
                    "Clear timed-out write");

        messages[0].Data[4] = 0xe2;
        count = 1;
        api.require(api.write(channel, messages, &count, 1000),
                    ERR_FAILED,
                    "Failed transmission indication");
        check(count == 0, "Failed transmission counted as sent");

        messages[0].Data[4] = 1;
        count = 1;
        api.require(api.write(channel, messages, &count, 1000),
                    STATUS_NOERROR,
                    "Write after failed transmission");
        PASSTHRU_MSG output[2] = {};
        count = 2;
        api.require(api.read(channel, output, &count, 100), ERR_TIMEOUT, "Partial timed Read");
        check(count == 1 && output[0].DataSize == 5, "Partial read count");
    }
    {
        Session session(api);
        J2534_ULONG channel = connect(api, session.id, CAN);
        PASSTHRU_MSG message = {};
        message.ProtocolID = CAN;
        message.TxFlags = CAN_29BIT_ID;
        message.DataSize = 5;
        putId(message.Data, 0x18da11f1);
        message.Data[4] = 0xe1;
        std::vector<PASSTHRU_MSG> batch(300, message);
        J2534_ULONG count = batch.size();
        api.require(api.write(channel, batch.data(), &count, 0),
                    ERR_BUFFER_FULL,
                    "Bound tracking when completion indications are lost");
        check(count > 0 && count < batch.size(), "Outstanding transmission limit");
        count = 1;
        api.require(api.write(channel, &message, &count, 0),
                    ERR_BUFFER_FULL,
                    "Full transmission tracking stays bounded");
        check(count == 0, "Full tracking accepted another message");
        J2534_ULONG periodic = 0x12345678;
        api.require(api.startPeriodic(channel, &message, &periodic, 100),
                    ERR_BUFFER_FULL,
                    "Periodic messages share the transmission tracking limit");
        check(periodic == 0x12345678, "Full tracking modified periodic ID");
        api.require(api.ioctl(channel, CLEAR_TX_BUFFER, NULL, NULL),
                    STATUS_NOERROR,
                    "Clear lost transmission records");
        message.Data[4] = 1;
        count = 1;
        api.require(api.write(channel, &message, &count, 1000),
                    STATUS_NOERROR,
                    "Write after clearing lost completion records");
        check(count == 1, "Recovered write count");
        printf("PASS bounded transmission tracking and recovery after lost indications\n");
    }
    for (J2534_ULONG fault = 0xff01; fault <= 0xff07; ++fault) {
        Session session(api);
        J2534_ULONG channel = connect(api, session.id, CAN);
        SCONFIG config = {fault, 123};
        SCONFIG_LIST list = {1, &config};
        J2534_LONG status = api.ioctl(channel, GET_CONFIG, &list, NULL);
        if (fault == 0xff06) {
            api.require(status, STATUS_NOERROR, "Response before EOF");
            platform::sleepMs(20);
            status = api.ioctl(channel, GET_CONFIG, &list, NULL);
        }
        api.require(status, ERR_DEVICE_NOT_CONNECTED, "Legacy wire fault");
        if (fault != 0xff06) {
            check(config.Value == 123, "Fault modified config output");
        }
        config.Parameter = LOOPBACK;
        api.require(api.ioctl(channel, GET_CONFIG, &list, NULL),
                    ERR_DEVICE_NOT_CONNECTED,
                    "Disconnect latch on local configuration");
        api.require(api.close(session.id), ERR_DEVICE_NOT_CONNECTED, "Close failed connection");
        session.id = 0;
        printf("PASS 04.04 wire fault 0x%04" J2534_PRIX "\n", fault);
    }
    printf("PASS 04.04 maximum payload, partial counts, write completion and error translation\n");
}

int main(int argc, char **argv) {
    try {
        if (argc != 3) {
            throw std::runtime_error("Expected library path and selftest/contract");
        }
        Api api(argv[1]);
        if (!strcmp(argv[2], "selftest")) {
            selfTest(api);
        } else {
            contractTest(api);
        }
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
