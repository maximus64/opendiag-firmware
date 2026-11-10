// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../platform/platform.h"
#include "j2534.pb.h"

struct Settings {
    std::string name;
    std::string transport;
    std::string host;
    std::string com;
    unsigned port;
    unsigned baud;
    unsigned rpcTimeout;
};

extern const void *moduleAnchor;
Settings loadSettings();

class Link {
    platform::Stream stream_;
    J2534_ULONG sequence_;
    Settings settings_;
    unsigned fragment_;
    void send(unsigned op,
              unsigned flags,
              unsigned offset,
              const unsigned char *data,
              unsigned size,
              uint32_t deadline);
    unsigned receive(unsigned op,
                     unsigned &flags,
                     unsigned &offset,
                     unsigned char *data,
                     uint32_t deadline);
    friend class HeldWait;
    void sendWake(unsigned op);

  public:
    Link();
    void open(const Settings &settings);
    void selectFrontend(bool recovering);
    void close();
    bool alive();
    // The firmware's fragment size from its capabilities, used both ways from then on.
    void setFragment(unsigned bytes);
    // waitMs extends the deadline for a request the device holds until it can answer.
    void rpc(const opendiag_Request &request, opendiag_Response &response, uint32_t waitMs = 0);
    // Any thread, at any time: while armed, a held wait in flight in this process, or
    // one that begins, is answered early, as though its timeout had run out.
    static void wakeWaits(bool armed);
};
