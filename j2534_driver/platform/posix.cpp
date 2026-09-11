// SPDX-License-Identifier: GPL-3.0-only
#include "platform.h"
#include <arpa/inet.h>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <dlfcn.h>
#include <fcntl.h>
#include <fstream>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdexcept>
#include <strings.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <termios.h>
#include <unistd.h>

namespace platform {
uint32_t milliseconds() {
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<uint32_t>(static_cast<uint64_t>(now.tv_sec) * 1000 + now.tv_nsec / 1000000);
}

void sleepMs(uint32_t duration) {
    timespec delay = {static_cast<time_t>(duration / 1000),
                      static_cast<long>(duration % 1000) * 1000000};
    while (nanosleep(&delay, &delay) < 0 && errno == EINTR) {
    }
}

uint32_t processId() {
    return static_cast<uint32_t>(getpid());
}

uint32_t threadId() {
    return static_cast<uint32_t>(syscall(SYS_gettid));
}

ErrorPreserver::ErrorPreserver()
    : saved_(errno) {
}

ErrorPreserver::~ErrorPreserver() {
    errno = static_cast<int>(saved_);
}

void utcTimestamp(char *text, size_t capacity) {
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    tm time;
    gmtime_r(&now.tv_sec, &time);
    format(text,
           capacity,
           "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
           time.tm_year + 1900,
           time.tm_mon + 1,
           time.tm_mday,
           time.tm_hour,
           time.tm_min,
           time.tm_sec,
           now.tv_nsec / 1000000);
}

std::string moduleDirectory(const void *anchor) {
    Dl_info info;
    if (!dladdr(anchor, &info) || !info.dli_fname) {
        throw std::runtime_error("Cannot locate the driver module");
    }
    char path[PATH_MAX];
    if (!realpath(info.dli_fname, path)) {
        throw std::runtime_error("Cannot resolve the driver path");
    }
    char *separator = strrchr(path, '/');
    separator[1] = 0;
    return path;
}

static std::string trim(const std::string &text) {
    size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

std::string readSetting(const std::string &path,
                        const char *section,
                        const char *key,
                        const char *fallback) {
    std::ifstream file(path.c_str());
    std::string line;
    bool selected = false;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }
        if (line[0] == '[' && line.back() == ']') {
            selected = !strcasecmp(trim(line.substr(1, line.size() - 2)).c_str(), section);
            continue;
        }
        size_t equals = line.find('=');
        if (!selected || equals == std::string::npos ||
            strcasecmp(trim(line.substr(0, equals)).c_str(), key)) {
            continue;
        }
        std::string value = trim(line.substr(equals + 1));
        if (value.size() >= 2 && (value[0] == '\'' || value[0] == '"') &&
            value.back() == value[0]) {
            value = value.substr(1, value.size() - 2);
        }
        if (value.size() >= 255) {
            throw std::runtime_error("Configuration value too long");
        }
        return value;
    }
    return fallback;
}

unsigned readNumber(const std::string &path,
                    const char *section,
                    const char *key,
                    unsigned fallback) {
    std::string text = readSetting(path, section, key, "");
    if (text.empty()) {
        return fallback;
    }
    char *end;
    errno = 0;
    int base = text.compare(0, 2, "0x") == 0 || text.compare(0, 2, "0X") == 0 ? 16 : 10;
    unsigned long value = strtoul(text.c_str(), &end, base);
    if (errno || end == text.c_str() || *end || text[0] == '-' || value > UINT_MAX) {
        return 0;
    }
    return static_cast<unsigned>(value);
}

Library openLibrary(const std::string &path) {
    return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
}

void *findSymbol(Library library, const char *name) {
    return dlsym(library, name);
}

void closeLibrary(Library library) {
    dlclose(library);
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

void appendLog(const char *path, const char *line, uint32_t maximumBytes) {
    int file = ::open(path, O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (file < 0) {
        return;
    }
    struct stat info;
    if (fstat(file, &info) || !S_ISREG(info.st_mode)) {
        ::close(file);
        return;
    }
    if (info.st_size >= maximumBytes) {
        ::close(file);
        char previous[PATH_MAX];
        if (format(previous, sizeof(previous), "%s.1", path) < 0 || rename(path, previous)) {
            return;
        }
        file = ::open(path, O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (file < 0) {
            return;
        }
    }
    size_t left = strlen(line);
    while (left) {
        ssize_t count = ::write(file, line, left);
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count <= 0) {
            break;
        }
        left -= static_cast<size_t>(count);
        line += count;
    }
    ::close(file);
}

Lease::Lease()
    : handle_(-1) {
}

Lease::~Lease() {
    if (handle_ >= 0) {
        ::close(static_cast<int>(handle_));
    }
}

bool Lease::acquire(const std::string &endpoint) {
    std::string identity = endpoint;
    char resolved[PATH_MAX];
    if (!endpoint.empty() && endpoint[0] == '/' && realpath(endpoint.c_str(), resolved)) {
        identity = resolved;
    }
    uint64_t hash = UINT64_C(14695981039346656037);
    for (unsigned char byte : identity) {
        hash ^= byte;
        hash *= UINT64_C(1099511628211);
    }
    char path[128];
    format(path,
           sizeof(path),
           "/tmp/opendiag-%u-%016llx.lock",
           static_cast<unsigned>(getuid()),
           static_cast<unsigned long long>(hash));
    int file = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (file < 0) {
        throw std::runtime_error("Cannot open the endpoint ownership file");
    }
    struct stat info;
    if (fstat(file, &info) || !S_ISREG(info.st_mode) || info.st_uid != getuid()) {
        ::close(file);
        throw std::runtime_error("Invalid endpoint ownership file");
    }
    if (flock(file, LOCK_EX | LOCK_NB)) {
        int error = errno;
        ::close(file);
        if (error == EWOULDBLOCK) {
            return false;
        }
        throw std::runtime_error("Cannot lock the endpoint");
    }
    // Keep the inode: unlinking a lock file can let a third process bypass an existing lock.
    handle_ = file;
    return true;
}

static bool waitReady(int file, short events, uint32_t timeout) {
    uint32_t deadline = milliseconds() + timeout;
    pollfd descriptor = {file, events, 0};
    for (;;) {
        int result = poll(&descriptor, 1, static_cast<int>(remaining(deadline)));
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result > 0 && (descriptor.revents & events)) {
            return true;
        }
        if (result == 0) {
            throw std::runtime_error("Transport I/O deadline expired");
        }
        throw std::runtime_error("Transport disconnected");
    }
}

void Stream::close() {
    if (socket_ >= 0) {
        ::close(static_cast<int>(socket_));
        socket_ = -1;
    }
    if (serial_ >= 0) {
        ::close(static_cast<int>(serial_));
        serial_ = -1;
    }
}

void Stream::openTcp(const std::string &host, unsigned port, uint32_t timeout) {
    close();
    socket_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {
        throw std::runtime_error("Socket creation failed");
    }
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
        throw std::runtime_error("Host must be an IPv4 address");
    }
    int file = static_cast<int>(socket_);
    if (connect(file, reinterpret_cast<sockaddr *>(&address), sizeof(address))) {
        if (errno != EINPROGRESS) {
            throw std::runtime_error("TCP connect failed");
        }
        waitReady(file, POLLOUT, timeout);
        int error = 0;
        socklen_t length = sizeof(error);
        if (getsockopt(file, SOL_SOCKET, SO_ERROR, &error, &length) || error) {
            throw std::runtime_error("TCP connection failed");
        }
    }
    int yes = 1;
    setsockopt(file, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
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

void Stream::openCom(const std::string &name, unsigned baud) {
    close();
    speed_t speed = serialSpeed(baud);
    serial_ = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (serial_ < 0) {
        throw std::runtime_error("Cannot open the configured serial device");
    }
    int file = static_cast<int>(serial_);
    termios settings;
    if (ioctl(file, TIOCEXCL) || tcgetattr(file, &settings)) {
        throw std::runtime_error("Cannot configure the serial device");
    }
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cc[VMIN] = 1;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, speed) || cfsetospeed(&settings, speed) ||
        tcsetattr(file, TCSANOW, &settings) || tcflush(file, TCIOFLUSH)) {
        throw std::runtime_error("Serial configuration failed");
    }
    int lines = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(file, TIOCMBIS, &lines) && errno != ENOTTY) {
        throw std::runtime_error("Cannot enable serial DTR/RTS");
    }
}

bool Stream::alive() {
    if (socket_ >= 0) {
        char byte;
        ssize_t count = recv(static_cast<int>(socket_), &byte, 1, MSG_PEEK | MSG_DONTWAIT);
        return count > 0 ||
               (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR));
    }
    if (serial_ >= 0) {
        pollfd descriptor = {static_cast<int>(serial_), POLLIN, 0};
        int result = poll(&descriptor, 1, 0);
        return (result >= 0 || errno == EINTR) &&
               !(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL));
    }
    return false;
}

uint32_t Stream::transferTcp(unsigned char *data, size_t size, uint32_t timeout, bool writing) {
    int file = static_cast<int>(socket_);
    waitReady(file, writing ? POLLOUT : POLLIN, timeout);
    ssize_t count;
    if (writing) {
        count = send(file, data, size, MSG_NOSIGNAL);
    } else {
        count = recv(file, data, size, 0);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        return 0;
    }
    if (count <= 0) {
        throw std::runtime_error("TCP disconnected");
    }
    return static_cast<uint32_t>(count);
}

uint32_t Stream::transferCom(unsigned char *data, size_t size, uint32_t timeout, bool writing) {
    int file = static_cast<int>(serial_);
    waitReady(file, writing ? POLLOUT : POLLIN, timeout);
    ssize_t count;
    if (writing) {
        count = ::write(file, data, size);
    } else {
        count = ::read(file, data, size);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        return 0;
    }
    if (count <= 0) {
        throw std::runtime_error("Serial device disconnected");
    }
    return static_cast<uint32_t>(count);
}
} // namespace platform
