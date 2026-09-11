// SPDX-License-Identifier: GPL-3.0-only
#include "platform.h"
#include <cstdio>
#include <stdexcept>
#include <windows.h>
#include <winsock2.h>

namespace platform {
uint32_t milliseconds() {
    return GetTickCount();
}

void sleepMs(uint32_t duration) {
    Sleep(duration);
}

uint32_t processId() {
    return GetCurrentProcessId();
}

uint32_t threadId() {
    return GetCurrentThreadId();
}

ErrorPreserver::ErrorPreserver()
    : saved_(GetLastError()) {
}

ErrorPreserver::~ErrorPreserver() {
    SetLastError(saved_);
}

void utcTimestamp(char *text, size_t capacity) {
    SYSTEMTIME time;
    GetSystemTime(&time);
    format(text,
           capacity,
           "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
           time.wYear,
           time.wMonth,
           time.wDay,
           time.wHour,
           time.wMinute,
           time.wSecond,
           time.wMilliseconds);
}

std::string moduleDirectory(const void *anchor) {
    HMODULE module = NULL;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            static_cast<const char *>(anchor),
                            &module)) {
        throw std::runtime_error("Cannot locate the driver module");
    }
    char path[MAX_PATH];
    DWORD length = GetModuleFileNameA(module, path, sizeof(path));
    if (!length || length >= sizeof(path)) {
        throw std::runtime_error("Module path too long");
    }
    char *separator = strrchr(path, '\\');
    if (!separator) {
        throw std::runtime_error("Invalid module path");
    }
    separator[1] = 0;
    return path;
}

std::string readSetting(const std::string &path,
                        const char *section,
                        const char *key,
                        const char *fallback) {
    char text[256];
    DWORD length =
        GetPrivateProfileStringA(section, key, fallback, text, sizeof(text), path.c_str());
    if (length >= sizeof(text) - 1) {
        throw std::runtime_error("Configuration value too long");
    }
    return text;
}

unsigned readNumber(const std::string &path,
                    const char *section,
                    const char *key,
                    unsigned fallback) {
    return GetPrivateProfileIntA(section, key, fallback, path.c_str());
}

Library openLibrary(const std::string &path) {
    return LoadLibraryExA(path.c_str(), NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
}

void *findSymbol(Library library, const char *name) {
    return reinterpret_cast<void *>(GetProcAddress(static_cast<HMODULE>(library), name));
}

void closeLibrary(Library library) {
    FreeLibrary(static_cast<HMODULE>(library));
}

const char *nativeLibraryName() {
    return "opendiag32.dll";
}

bool readValue(const J2534_ULONG *source, J2534_ULONG &value) {
    SIZE_T count = 0;
    return source &&
           ReadProcessMemory(GetCurrentProcess(), source, &value, sizeof(value), &count) &&
           count == sizeof(value);
}

void appendLog(const char *path, const char *line, uint32_t maximumBytes) {
    HANDLE file = CreateFileA(path,
                              FILE_APPEND_DATA,
                              FILE_SHARE_READ | FILE_SHARE_DELETE,
                              NULL,
                              OPEN_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL);
    if (file == INVALID_HANDLE_VALUE) {
        return;
    }
    LARGE_INTEGER size;
    if (!GetFileSizeEx(file, &size)) {
        CloseHandle(file);
        return;
    }
    if (size.QuadPart >= maximumBytes) {
        CloseHandle(file);
        char previous[MAX_PATH];
        if (format(previous, sizeof(previous), "%s.1", path) < 0 ||
            !MoveFileExA(path, previous, MOVEFILE_REPLACE_EXISTING)) {
            return;
        }
        file = CreateFileA(path,
                           FILE_APPEND_DATA,
                           FILE_SHARE_READ | FILE_SHARE_DELETE,
                           NULL,
                           OPEN_ALWAYS,
                           FILE_ATTRIBUTE_NORMAL,
                           NULL);
        if (file == INVALID_HANDLE_VALUE) {
            return;
        }
    }
    DWORD count;
    WriteFile(file, line, static_cast<DWORD>(strlen(line)), &count, NULL);
    CloseHandle(file);
}

Lease::Lease()
    : handle_(-1) {
}

Lease::~Lease() {
    if (handle_ != -1) {
        CloseHandle(reinterpret_cast<HANDLE>(handle_));
    }
}

bool Lease::acquire(const std::string &endpoint) {
    uint32_t hash = 2166136261U;
    for (unsigned char byte : endpoint) {
        hash ^= static_cast<unsigned char>(toupper(byte));
        hash *= 16777619U;
    }
    char name[80];
    format(name, sizeof(name), "Local\\OpenDIAG-J2534-05-%08X", static_cast<unsigned>(hash));
    HANDLE event = CreateEventA(NULL, TRUE, FALSE, name);
    if (!event) {
        throw std::runtime_error("Device ownership event failed");
    }
    bool acquired = GetLastError() != ERROR_ALREADY_EXISTS;
    if (!acquired) {
        CloseHandle(event);
        return false;
    }
    handle_ = reinterpret_cast<intptr_t>(event);
    return true;
}

static HANDLE asHandle(intptr_t value) {
    return reinterpret_cast<HANDLE>(value);
}

void Stream::close() {
    if (socket_ != -1) {
        closesocket(socket_);
        socket_ = -1;
    }

    if (serial_ != -1) {
        CloseHandle(asHandle(serial_));
        serial_ = -1;
    }

    if (winsock_) {
        WSACleanup();
        winsock_ = false;
    }
}

void Stream::openTcp(const std::string &host, unsigned port, uint32_t timeout) {
    close();
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data)) {
        throw std::runtime_error("WSAStartup failed");
    }

    winsock_ = true;
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == -1) {
        throw std::runtime_error("socket creation failed");
    }

    u_long nonblocking = 1;
    if (ioctlsocket(socket_, FIONBIO, &nonblocking)) {
        throw std::runtime_error("nonblocking socket failed");
    }

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons((u_short)port);
    address.sin_addr.s_addr = inet_addr(host.c_str());
    if (address.sin_addr.s_addr == INADDR_NONE) {
        throw std::runtime_error("Host must be an IPv4 address");
    }

    if (connect(socket_, (sockaddr *)&address, sizeof(address)) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            throw std::runtime_error("TCP connect failed");
        }
        fd_set writable;
        fd_set errors;
        FD_ZERO(&writable);
        FD_SET(socket_, &writable);
        FD_ZERO(&errors);
        FD_SET(socket_, &errors);
        timeval wait = {(long)(timeout / 1000), (long)(timeout % 1000) * 1000};
        if (select(0, NULL, &writable, &errors, &wait) <= 0 || FD_ISSET(socket_, &errors)) {
            throw std::runtime_error("TCP connect timed out or refused");
        }
        int error = 0;
        int errorLength = sizeof(error);
        if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, (char *)&error, &errorLength) || error) {
            throw std::runtime_error("TCP connection failed");
        }
    }

    BOOL yes = TRUE;
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (char *)&yes, sizeof(yes));
}

void Stream::openCom(const std::string &name, unsigned baud) {
    close();
    std::string path = name.compare(0, 4, "\\\\.\\") == 0 ? name : "\\\\.\\" + name;
    serial_ = reinterpret_cast<intptr_t>(CreateFileA(path.c_str(),
                                                     GENERIC_READ | GENERIC_WRITE,
                                                     0,
                                                     NULL,
                                                     OPEN_EXISTING,
                                                     FILE_FLAG_OVERLAPPED,
                                                     NULL));
    if (serial_ == -1) {
        throw std::runtime_error("cannot open configured COM port");
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(asHandle(serial_), &dcb)) {
        throw std::runtime_error("GetCommState failed");
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fAbortOnError = FALSE;
    COMMTIMEOUTS timeouts = {};
    if (!SetCommState(asHandle(serial_), &dcb) || !SetCommTimeouts(asHandle(serial_), &timeouts) ||
        !PurgeComm(asHandle(serial_), PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        throw std::runtime_error("COM port configuration failed");
    }
}

bool Stream::alive() {
    if (socket_ != -1) {
        char byte;
        int rc = recv(socket_, &byte, 1, MSG_PEEK);
        return rc > 0 || (rc == SOCKET_ERROR && WSAGetLastError() == WSAEWOULDBLOCK);
    }

    if (serial_ != -1) {
        DWORD errors;
        COMSTAT status;
        return ClearCommError(asHandle(serial_), &errors, &status) != 0;
    }

    return false;
}

uint32_t Stream::transferTcp(unsigned char *data, size_t size, uint32_t timeout, bool writing) {
    fd_set ready;
    FD_ZERO(&ready);
    FD_SET(socket_, &ready);
    timeval wait = {(long)(timeout / 1000), (long)(timeout % 1000) * 1000};

    int result;
    if (writing) {
        result = select(0, NULL, &ready, NULL, &wait);
    } else {
        result = select(0, &ready, NULL, NULL, &wait);
    }

    if (result <= 0) {
        throw std::runtime_error("TCP I/O deadline or connection error");
    }

    if (writing) {
        result = send(socket_, (const char *)data, (int)size, 0);
    } else {
        result = recv(socket_, (char *)data, (int)size, 0);
    }

    if (result == SOCKET_ERROR && WSAGetLastError() == WSAEWOULDBLOCK) {
        return 0;
    }

    if (result <= 0) {
        throw std::runtime_error("TCP disconnected");
    }

    return (DWORD)result;
}

uint32_t Stream::transferCom(unsigned char *data, size_t size, uint32_t timeout, bool writing) {
    OVERLAPPED operation = {};
    operation.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!operation.hEvent) {
        throw std::runtime_error("COM event allocation failed");
    }

    DWORD transferred = 0;
    BOOL completed;
    if (writing) {
        completed = WriteFile(asHandle(serial_), data, (DWORD)size, &transferred, &operation);
    } else {
        completed = ReadFile(asHandle(serial_), data, (DWORD)size, &transferred, &operation);
    }

    if (!completed && GetLastError() == ERROR_IO_PENDING) {
        DWORD waitResult = WaitForSingleObject(operation.hEvent, timeout);
        if (waitResult == WAIT_OBJECT_0) {
            completed = GetOverlappedResult(asHandle(serial_), &operation, &transferred, FALSE);
        } else {
            // Cancellation must finish before the stack-owned OVERLAPPED and its event go away.
            CancelIo(asHandle(serial_));
            GetOverlappedResult(asHandle(serial_), &operation, &transferred, TRUE);
            completed = FALSE;
        }
    }

    CloseHandle(operation.hEvent);
    if (!completed || transferred == 0) {
        throw std::runtime_error("COM I/O deadline or disconnect");
    }

    return transferred;
}

} // namespace platform
