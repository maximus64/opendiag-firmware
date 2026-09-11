// SPDX-License-Identifier: GPL-3.0-only
#include "../common/api_trace.h"
#include "../platform/platform.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

static unsigned checks;
static int moduleAnchor;

static void check(bool condition, const char *message) {
    ++checks;
    if (!condition) {
        throw std::runtime_error(message);
    }
}

static void testSerial() {
    int master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
    check(master >= 0 && !grantpt(master) && !unlockpt(master), "Create pseudo terminal");
    std::string slave = ptsname(master);
    platform::Stream stream;
    stream.openCom(slave, 115200);
    check(stream.alive(), "Serial connection is alive");
    termios settings;
    check(!tcgetattr(master, &settings), "Read terminal configuration");
    check(cfgetispeed(&settings) == B115200 && !(settings.c_lflag & (ICANON | ECHO)),
          "Serial raw mode and baud rate");
    check((settings.c_cflag & CSIZE) == CS8 && !(settings.c_cflag & (PARENB | CSTOPB | CRTSCTS)),
          "Serial 8N1 without flow control");
    unsigned char sent[] = {0, 0x0a, 0x0d, 0x11, 0x13, 0x80, 0xff};
    stream.write(sent, sizeof(sent), platform::milliseconds() + 1000);
    unsigned char received[sizeof(sent)] = {};
    check(::read(master, received, sizeof(received)) == sizeof(received) &&
              !memcmp(sent, received, sizeof(sent)),
          "Serial write preserves binary bytes");
    std::thread producer([&]() {
        for (unsigned char byte : sent) {
            if (::write(master, &byte, 1) != 1) {
                return;
            }
            platform::sleepMs(2);
        }
    });
    try {
        stream.read(received, sizeof(received), platform::milliseconds() + 1000);
    } catch (...) {
        producer.join();
        throw;
    }
    producer.join();
    check(!memcmp(sent, received, sizeof(sent)), "Serial exact read handles fragments");
    uint32_t start = platform::milliseconds();
    bool timedOut = false;
    try {
        stream.read(received, 1, start + 50);
    } catch (const std::runtime_error &) {
        timedOut = true;
    }
    uint32_t elapsed = platform::milliseconds() - start;
    check(timedOut && elapsed >= 50 && elapsed < 500, "Serial read deadline");
    ::close(master);
    check(!stream.alive(), "Serial hangup detected");
    bool disconnected = false;
    try {
        stream.read(received, 1, platform::milliseconds() + 1000);
    } catch (const std::runtime_error &) {
        disconnected = true;
    }
    check(disconnected, "Serial hangup rejects I/O");
    stream.close();
}

static void testLease() {
    std::string endpoint = "test-endpoint-" + std::to_string(getpid());
    platform::Lease second;
    {
        platform::Lease first;
        check(first.acquire(endpoint), "Acquire endpoint lease");
        check(!second.acquire(endpoint), "Reject a second lease in this process");
        pid_t child = fork();
        check(child >= 0, "Fork lease checker");
        if (!child) {
            platform::Lease otherProcess;
            _exit(otherProcess.acquire(endpoint) ? 1 : 0);
        }
        int status;
        check(waitpid(child, &status, 0) == child && WIFEXITED(status) && !WEXITSTATUS(status),
              "Reject another process owning the endpoint");
    }
    check(second.acquire(endpoint), "Lease released when the owner closes");
}

static void testLogging() {
    const J2534_ULONG value = 0x12345678;
    J2534_ULONG result = 0;
    check(platform::readValue(&value, result) && result == value, "Read valid logging pointer");
    auto invalid = reinterpret_cast<const J2534_ULONG *>(1);
    check(!platform::readValue(invalid, result), "Reject invalid logging pointer");
    errno = E2BIG;
    {
        diagnostics::ApiTrace trace(&moduleAnchor,
                                    "05.00",
                                    "Probe",
                                    "value_out=%p",
                                    static_cast<const void *>(&value));
        trace.inputValue("bad", invalid);
        trace.outputValue("value", &value);
        trace.outputValue("bad", invalid);
        check(trace.result(0) == 0, "Logging preserves result");
    }
    check(errno == E2BIG, "Logging preserves errno");
    std::string path = platform::moduleDirectory(&moduleAnchor) + "opendiag-05.00-" +
                       std::to_string(getpid()) + ".log";
    std::ifstream input(path.c_str());
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    check(text.find("<unreadable>") != std::string::npos &&
              text.find("0x12345678") != std::string::npos,
          "Logging records values and invalid pointers");
    input.close();
    platform::appendLog(path.c_str(), "record\n", 1);
    check(access((path + ".1").c_str(), F_OK) == 0, "Log rotation creates one backup");
    std::ifstream rotated(path.c_str());
    std::string current((std::istreambuf_iterator<char>(rotated)),
                        std::istreambuf_iterator<char>());
    check(current == "record\n", "Log rotation starts a fresh file");
    platform::appendLog(platform::moduleDirectory(&moduleAnchor).c_str(), "ignored", 1);
    unlink(path.c_str());
    unlink((path + ".1").c_str());
}

int main() {
    try {
        testSerial();
        testLease();
        testLogging();
        printf("PASS %u platform checks: serial, deadlines, ownership and logging\n", checks);
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
