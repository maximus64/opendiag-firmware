// SPDX-License-Identifier: GPL-3.0-only
#include "link.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <algorithm>
#include <stdexcept>
#include <stdio.h>

const void *moduleAnchor = &moduleAnchor;

static const unsigned WIRE_HEADER_BYTES = 16;
static const unsigned WIRE_CRC_BYTES = 4;
// Every firmware takes fragments this large; its capabilities may allow more.
static const unsigned WIRE_FRAGMENT_DEFAULT = 192;
static const unsigned WIRE_RPC_BYTES = 4608;
// No fragment is larger than a whole RPC.
static const unsigned WIRE_FRAME_BYTES = WIRE_HEADER_BYTES + WIRE_RPC_BYTES + WIRE_CRC_BYTES;
static const unsigned WIRE_MORE = 0x01;
static const unsigned WIRE_FETCH = 0x02;
static const unsigned WIRE_ACK = 0x04;
static const unsigned WIRE_WAKE = 0x08;
static const unsigned WIRE_RESPONSE = 0x80;

static std::string setting(const char *key, const char *fallback, const std::string &path) {
    return platform::readSetting(path, "device", key, fallback);
}

Settings loadSettings() {
    const std::string path = platform::configPath();
    Settings settings;
    settings.name = setting("Name", "OpenDIAG", path);
    settings.transport = setting("Transport", "com", path);
    settings.host = setting("Host", "192.168.56.1", path);
    settings.com = setting("COM", "COM3", path);
    settings.port = platform::readNumber(path, "device", "Port", 23200);
    settings.baud = platform::readNumber(path, "device", "Baud", 115200);
    settings.rpcTimeout = platform::readNumber(path, "device", "RpcTimeout", 8000);
    bool validName = !settings.name.empty() && settings.name.size() <= 79;
    bool validTimeout = settings.rpcTimeout >= 100 && settings.rpcTimeout <= 30000;
    bool validDataPort = settings.port > 0 && settings.port <= 65535;
    bool validTransport = settings.transport == "tcp" || settings.transport == "com";
    if (!validName || !validTimeout || !validDataPort || !validTransport) {
        throw std::runtime_error("invalid opendiag.ini settings");
    }

    for (size_t i = 0; i < settings.name.size(); ++i) {
        if ((unsigned char)settings.name[i] < 32 || (unsigned char)settings.name[i] > 126) {
            throw std::runtime_error("device name must be ASCII");
        }
    }

    return settings;
}

static unsigned get16(const unsigned char *p) {
    return p[0] | (unsigned)p[1] << 8;
}

static J2534_ULONG get32(const unsigned char *p) {
    return p[0] | (J2534_ULONG)p[1] << 8 | (J2534_ULONG)p[2] << 16 | (J2534_ULONG)p[3] << 24;
}

static void put16(unsigned char *p, unsigned v) {
    p[0] = (unsigned char)v;
    p[1] = (unsigned char)(v >> 8);
}

static void put32(unsigned char *p, J2534_ULONG v) {
    for (unsigned i = 0; i < 4; ++i) {
        p[i] = (unsigned char)(v >> (i * 8));
    }
}

static J2534_ULONG crc32(const unsigned char *p, unsigned size) {
    J2534_ULONG crc = 0xffffffffUL;
    while (size--) {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; ++i) {
            crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1)));
        }
    }

    return ~crc;
}

// The held wait in flight, if any, and whether another caller wants it ended. One lock
// orders a wait beginning against a wake being armed, so neither can miss the other,
// and keeps the link alive while a wake is written to it.
static platform::Mutex waitLock;
static Link *waitingLink = NULL;
static unsigned waitingOp = 0;
static bool wakeArmed = false;

class HeldWait {
    bool active_;

  public:
    HeldWait()
        : active_(false) {
    }
    void begin(Link *link, unsigned op) {
        platform::MutexLock lock(waitLock);
        waitingLink = link;
        waitingOp = op;
        active_ = true;
        if (wakeArmed) {
            link->sendWake(op);
        }
    }
    void end() {
        if (active_) {
            platform::MutexLock lock(waitLock);
            waitingLink = NULL;
            active_ = false;
        }
    }
    ~HeldWait() {
        end();
    }
};

void Link::wakeWaits(bool armed) {
    platform::MutexLock lock(waitLock);
    wakeArmed = armed;
    if (armed && waitingLink) {
        waitingLink->sendWake(waitingOp);
    }
}

// The waiting thread is blocked reading; this only writes, and failures surface there.
void Link::sendWake(unsigned op) {
    try {
        send(op, WIRE_WAKE, 0, NULL, 0, platform::milliseconds() + settings_.rpcTimeout);
    } catch (const std::exception &) {
    }
}

Link::Link()
    : sequence_(0),
      fragment_(WIRE_FRAGMENT_DEFAULT) {
}

void Link::setFragment(unsigned bytes) {
    fragment_ = std::min(bytes, WIRE_RPC_BYTES);
}

void Link::close() {
    stream_.close();
}

bool Link::alive() {
    return stream_.alive();
}

void Link::open(const Settings &settings) {
    settings_ = settings;
    fragment_ = WIRE_FRAGMENT_DEFAULT;
    if (settings.transport == "tcp") {
        stream_.openTcp(settings.host, settings.port, settings.rpcTimeout);
    } else {
        stream_.openCom(settings.com, settings.baud);
    }

    selectFrontend(false);
}

void Link::selectFrontend(bool recovering) {
    // On recovery, exceed the firmware's two-second partial-frame timeout.
    stream_.drain(recovering ? 2100 : 50);
    const char command[] = "AT VIF PASSTHRU\r";
    stream_.write((const unsigned char *)command,
                  sizeof(command) - 1,
                  platform::milliseconds() + settings_.rpcTimeout);
    // ELM replies with echo/OK/prompt; an existing binary frontend stays silent.
    stream_.drain(300);
}

void Link::send(unsigned op,
                unsigned flags,
                unsigned offset,
                const unsigned char *data,
                unsigned size,
                uint32_t deadline) {
    // The CRC covers the header after its four-byte magic, followed by the payload.
    unsigned char frame[WIRE_FRAME_BYTES] = {'J', '2', '5', '3', 1};
    frame[5] = (unsigned char)op;
    put16(frame + 6, flags);
    put32(frame + 8, sequence_);
    put16(frame + 12, offset);
    put16(frame + 14, size);
    if (size) {
        memcpy(frame + WIRE_HEADER_BYTES, data, size);
    }

    put32(frame + WIRE_HEADER_BYTES + size, crc32(frame + 4, size + 12));
    stream_.write(frame, size + WIRE_HEADER_BYTES + WIRE_CRC_BYTES, deadline);
}

unsigned Link::receive(unsigned op,
                       unsigned &flags,
                       unsigned &offset,
                       unsigned char *data,
                       uint32_t deadline) {
    unsigned char frame[WIRE_FRAME_BYTES];
    stream_.read(frame, WIRE_HEADER_BYTES, deadline);
    unsigned size = get16(frame + 14);
    if (memcmp(frame, "J253", 4) || frame[4] != 1 || frame[5] != op ||
        get32(frame + 8) != sequence_ || size > fragment_) {
        throw std::runtime_error("invalid J2534 wire header");
    }

    stream_.read(frame + WIRE_HEADER_BYTES, size + WIRE_CRC_BYTES, deadline);
    if (get32(frame + WIRE_HEADER_BYTES + size) != crc32(frame + 4, size + 12)) {
        throw std::runtime_error("J2534 wire CRC mismatch");
    }

    flags = get16(frame + 6);
    offset = get16(frame + 12);
    memcpy(data, frame + WIRE_HEADER_BYTES, size);
    return size;
}

void Link::rpc(const opendiag_Request &request, opendiag_Response &response, uint32_t waitMs) {
    unsigned char encoded[WIRE_RPC_BYTES];
    unsigned char decoded[WIRE_RPC_BYTES];
    unsigned char fragment[WIRE_RPC_BYTES];
    pb_ostream_t output = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!request.which_command || !pb_encode(&output, opendiag_Request_fields, &request)) {
        throw std::runtime_error("protobuf request encoding failed");
    }
    ++sequence_;
    unsigned op = request.which_command - 1;
    unsigned flags = 0;
    unsigned gotOffset = 0;
    unsigned gotSize = 0;
    uint32_t deadline = platform::milliseconds() + settings_.rpcTimeout + waitMs;

    // Intermediate request fragments receive ACKs; the last fragment starts the response,
    // which for a held request is the wait another caller may cut short.
    HeldWait held;
    for (unsigned offset = 0; offset < output.bytes_written;) {
        unsigned size =
            (unsigned)std::min((size_t)fragment_, output.bytes_written - offset);
        bool more = offset + size < output.bytes_written;
        send(op, more ? WIRE_MORE : 0, offset, encoded + offset, size, deadline);
        if (!more && waitMs) {
            held.begin(this, op);
        }
        gotSize = receive(op, flags, gotOffset, fragment, deadline);
        held.end();
        offset += size;
        if (more && (flags != (WIRE_RESPONSE | WIRE_ACK) || gotOffset != offset || gotSize)) {
            throw std::runtime_error("invalid fragment acknowledgement");
        }
    }

    // Fetch each remaining response fragment before decoding the complete protobuf message.
    unsigned total = 0;
    for (;;) {
        bool finalFragment = flags == WIRE_RESPONSE;
        bool moreFragments = flags == (WIRE_RESPONSE | WIRE_MORE);
        bool validFlags = finalFragment || moreFragments;
        bool validOffset = gotOffset == total;
        bool fitsResponse = total + gotSize <= sizeof(decoded);
        bool emptyContinuation = moreFragments && gotSize == 0;
        if (!validFlags || !validOffset || !fitsResponse || emptyContinuation) {
            throw std::runtime_error("invalid fragmented response");
        }
        memcpy(decoded + total, fragment, gotSize);
        total += gotSize;
        if (finalFragment) {
            break;
        }
        send(op, WIRE_FETCH, total, NULL, 0, deadline);
        gotSize = receive(op, flags, gotOffset, fragment, deadline);
    }

    memset(&response, 0, sizeof(response));
    pb_istream_t input = pb_istream_from_buffer(decoded, total);
    if (!pb_decode(&input, opendiag_Response_fields, &response)) {
        throw std::runtime_error("protobuf response decoding failed");
    }
}
