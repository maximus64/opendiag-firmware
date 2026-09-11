// SPDX-License-Identifier: GPL-3.0-only
#include "platform.h"
#include <algorithm>
#include <cstdio>
#include <stdexcept>

namespace platform {
int formatV(char *text, size_t capacity, const char *pattern, va_list arguments) {
    if (!capacity) {
        return -1;
    }
    int count = vsnprintf(text, capacity, pattern, arguments);
    text[capacity - 1] = 0;
    if (count < 0 || static_cast<size_t>(count) >= capacity) {
        return -1;
    }
    return count;
}

int format(char *text, size_t capacity, const char *pattern, ...) {
    va_list arguments;
    va_start(arguments, pattern);
    int count = formatV(text, capacity, pattern, arguments);
    va_end(arguments);
    return count;
}

void copyText(char *destination, size_t capacity, const char *source) {
    if (capacity) {
        size_t count = std::min(capacity - 1, strlen(source));
        memcpy(destination, source, count);
        destination[count] = 0;
    }
}

std::string configPath(const void *anchor) {
    return moduleDirectory(anchor) + "opendiag.ini";
}

uint32_t remaining(uint32_t deadline) {
    int32_t value = static_cast<int32_t>(deadline - milliseconds());
    if (value <= 0) {
        throw std::runtime_error("transport deadline expired");
    }
    return static_cast<uint32_t>(value);
}

Stream::Stream()
    : socket_(-1),
      serial_(-1),
      winsock_(false) {
}

Stream::~Stream() {
    close();
}

void Stream::transfer(unsigned char *data, size_t size, uint32_t deadline, bool writing) {
    while (size) {
        uint32_t timeout = remaining(deadline);
        uint32_t transferred;
        if (socket_ != -1) {
            transferred = transferTcp(data, size, timeout, writing);
        } else if (serial_ != -1) {
            transferred = transferCom(data, size, timeout, writing);
        } else {
            throw std::runtime_error("transport is closed");
        }
        data += transferred;
        size -= transferred;
    }
}

void Stream::write(const unsigned char *data, size_t size, uint32_t deadline) {
    transfer(const_cast<unsigned char *>(data), size, deadline, true);
}

void Stream::read(unsigned char *data, size_t size, uint32_t deadline) {
    transfer(data, size, deadline, false);
}

void Stream::drain(uint32_t duration) {
    uint32_t deadline = milliseconds() + duration;
    while (static_cast<int32_t>(deadline - milliseconds()) > 0) {
        unsigned char byte;
        try {
            read(&byte, 1, deadline);
        } catch (const std::exception &) {
            break;
        }
    }
}
} // namespace platform
