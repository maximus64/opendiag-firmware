// SPDX-License-Identifier: GPL-3.0-only
#include "platform.h"
#include "posix.h"
#include <IOKit/serial/ioss.h>
#include <fcntl.h>
#include <mach/mach.h>
#include <mach/mach_vm.h>
#include <pthread.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace platform {
uint32_t threadId() {
    uint64_t id = 0;
    pthread_threadid_np(NULL, &id);
    return static_cast<uint32_t>(id);
}

const char *nativeLibraryName() {
    return "libopendiag.dylib";
}

bool readValue(const J2534_ULONG *source, J2534_ULONG &value) {
    if (!source) {
        return false;
    }
    mach_vm_size_t size = 0;
    return mach_vm_read_overwrite(mach_task_self(),
                                  reinterpret_cast<mach_vm_address_t>(source),
                                  sizeof(value),
                                  reinterpret_cast<mach_vm_address_t>(&value),
                                  &size) == KERN_SUCCESS &&
           size == sizeof(value);
}

int openTcpSocket() {
    int file = socket(AF_INET, SOCK_STREAM, 0);
    if (file >= 0 && (fcntl(file, F_SETFD, FD_CLOEXEC) || fcntl(file, F_SETFL, O_NONBLOCK))) {
        ::close(file);
        return -1;
    }
    return file;
}

// Darwin speeds are plain numbers, but termios stops at B230400. Faster rates go
// through IOSSIOSPEED, which serial drivers implement and pseudo terminals do not.
void configureSerial(int file, termios &settings, unsigned baud) {
    if (!baud) {
        throw std::runtime_error("Unsupported serial baud rate");
    }
    speed_t speed = baud;
    bool standard = speed <= B230400;
    if ((standard && (cfsetispeed(&settings, speed) || cfsetospeed(&settings, speed))) ||
        tcsetattr(file, TCSANOW, &settings) ||
        (!standard && ioctl(file, IOSSIOSPEED, &speed))) {
        throw std::runtime_error("Serial configuration failed");
    }
}
} // namespace platform
