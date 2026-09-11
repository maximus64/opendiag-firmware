// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../include/j2534_types.h"
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstring>
#include <string>

namespace platform {
using Library = void *;

uint32_t milliseconds();
void sleepMs(uint32_t duration);
uint32_t remaining(uint32_t deadline);
uint32_t processId();
uint32_t threadId();
void utcTimestamp(char *text, size_t capacity);

class ErrorPreserver {
    unsigned long saved_;

  public:
    ErrorPreserver();
    ~ErrorPreserver();
};

int format(char *text, size_t capacity, const char *pattern, ...)
    __attribute__((format(printf, 3, 4)));
int formatV(char *text, size_t capacity, const char *pattern, va_list arguments);
void copyText(char *destination, size_t capacity, const char *source);
bool readValue(const J2534_ULONG *source, J2534_ULONG &value);
void appendLog(const char *path, const char *line, uint32_t maximumBytes);

std::string moduleDirectory(const void *anchor);
std::string configPath(const void *anchor);
std::string readSetting(const std::string &path,
                        const char *section,
                        const char *key,
                        const char *fallback);
unsigned readNumber(const std::string &path,
                    const char *section,
                    const char *key,
                    unsigned fallback);
Library openLibrary(const std::string &path);
void *findSymbol(Library library, const char *name);
void closeLibrary(Library library);
const char *nativeLibraryName();

class Lease {
    intptr_t handle_;

  public:
    Lease();
    ~Lease();
    bool acquire(const std::string &endpoint);
    Lease(const Lease &) = delete;
    Lease &operator=(const Lease &) = delete;
};

class Stream {
    intptr_t socket_;
    intptr_t serial_;
    bool winsock_;
    uint32_t transferTcp(unsigned char *data, size_t size, uint32_t timeout, bool writing);
    uint32_t transferCom(unsigned char *data, size_t size, uint32_t timeout, bool writing);
    void transfer(unsigned char *data, size_t size, uint32_t deadline, bool writing);

  public:
    Stream();
    ~Stream();
    void openTcp(const std::string &host, unsigned port, uint32_t timeout);
    void openCom(const std::string &name, unsigned baud);
    void close();
    bool alive();
    void write(const unsigned char *data, size_t size, uint32_t deadline);
    void read(unsigned char *data, size_t size, uint32_t deadline);
    void drain(uint32_t duration);
    Stream(const Stream &) = delete;
    Stream &operator=(const Stream &) = delete;
};
} // namespace platform
