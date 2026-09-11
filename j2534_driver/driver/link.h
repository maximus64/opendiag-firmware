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
std::string configPath();
Settings loadSettings();

class Link {
    platform::Stream stream_;
    J2534_ULONG sequence_;
    Settings settings_;
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

  public:
    Link();
    void open(const Settings &settings);
    void selectFrontend(bool recovering);
    void close();
    bool alive();
    void rpc(const opendiag_Request &request, opendiag_Response &response);
};
