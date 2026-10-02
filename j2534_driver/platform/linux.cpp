// SPDX-License-Identifier: GPL-3.0-only
#include "platform.h"
#include "posix.h"
#include <stdexcept>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>

namespace platform {
uint32_t threadId() {
    return static_cast<uint32_t>(syscall(SYS_gettid));
}

const char *nativeLibraryName() {
    return "libopendiag.so";
}

bool readValue(const J2534_ULONG *source, J2534_ULONG &value) {
    if (!source) {
        return false;
    }
    iovec local = {&value, sizeof(value)};
    iovec remote = {const_cast<J2534_ULONG *>(source), sizeof(value)};
    return process_vm_readv(getpid(), &local, 1, &remote, 1, 0) == sizeof(value);
}

int openTcpSocket() {
    return socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
}

static speed_t serialSpeed(unsigned baud) {
    struct Speed {
        unsigned value;
        speed_t setting;
    };
    static const Speed speeds[] = {
        {300, B300},         {600, B600},         {1200, B1200},       {2400, B2400},
        {4800, B4800},       {9600, B9600},       {19200, B19200},     {38400, B38400},
        {57600, B57600},     {115200, B115200},   {230400, B230400},   {460800, B460800},
        {500000, B500000},   {576000, B576000},   {921600, B921600},   {1000000, B1000000},
        {1152000, B1152000}, {1500000, B1500000}, {2000000, B2000000}, {2500000, B2500000},
        {3000000, B3000000}, {3500000, B3500000}, {4000000, B4000000}};
    for (const Speed &speed : speeds) {
        if (speed.value == baud) {
            return speed.setting;
        }
    }
    throw std::runtime_error("Unsupported serial baud rate");
}

void configureSerial(int file, termios &settings, unsigned baud) {
    speed_t speed = serialSpeed(baud);
    if (cfsetispeed(&settings, speed) || cfsetospeed(&settings, speed) ||
        tcsetattr(file, TCSANOW, &settings)) {
        throw std::runtime_error("Serial configuration failed");
    }
}
} // namespace platform
