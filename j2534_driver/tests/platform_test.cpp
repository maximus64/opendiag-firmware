// SPDX-License-Identifier: GPL-3.0-only
#include "../common/api_trace.h"
#include "../platform/platform.h"
#include "../platform/posix.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <limits.h>
#include <pwd.h>
#include <stdexcept>
#include <sys/stat.h>
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
    {
        // A second terminal, so the probe's exclusive lock cannot touch the working stream.
        int probe = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
        check(probe >= 0 && !grantpt(probe) && !unlockpt(probe), "Create probe terminal");
        platform::Stream unsupported;
        bool rejectedBaud = false;
        try {
            unsupported.openCom(ptsname(probe), 12345);
        } catch (const std::runtime_error &error) {
            rejectedBaud = !strcmp(error.what(), "Unsupported serial baud rate");
        }
        check(rejectedBaud, "Unsupported serial baud rate is rejected");
        ::close(probe);
    }
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
    {
        // A trace that never receives its result records the abort when it ends.
        diagnostics::ApiTrace abandoned(&moduleAnchor, "05.00", "Abandoned", "value=%u", 1u);
    }
    std::string path = platform::logDirectory(&moduleAnchor) + "opendiag-05.00-" +
                       std::to_string(getpid()) + ".log";
    std::ifstream input(path.c_str());
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    check(text.find("<unreadable>") != std::string::npos &&
              text.find("0x12345678") != std::string::npos,
          "Logging records values and invalid pointers");
    check(text.find("ABORT no return recorded") != std::string::npos,
          "A trace without a result records its abort");
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

static void testPaths() {
    const std::string directory = platform::moduleDirectory(&moduleAnchor);
    const std::string temporary = directory + "path-test-" + std::to_string(getpid()) + "/";
    check(platform::createDirectories(temporary), "Create configuration test folder");
    const std::string system = temporary + "system.ini";
    const std::string overridePath = temporary + "override.ini";
    auto rejectsOverride = [&](const std::string &path) {
        bool rejected = false;
        try {
            platform::selectConfigPath(path, system);
        } catch (const std::runtime_error &error) {
            const std::string message = error.what();
            check(message.find("OPENDIAG_DRIVER_CONFIG file not found:") != std::string::npos &&
                      message.find(platform::absolutePath(path)) != std::string::npos,
                  "Override error identifies the environment variable and resolved path");
            rejected = true;
        }
        check(rejected, "Invalid explicit configuration fails instead of falling back");
    };
    auto rejectsSystem = [&](const std::string &path, const std::string &expected) {
        bool rejected = false;
        try {
            platform::selectConfigPath("", path);
        } catch (const std::runtime_error &error) {
            check(error.what() == expected, "System configuration error identifies the problem");
            rejected = true;
        }
        check(rejected, "Invalid system configuration fails without legacy or default settings");
    };
    rejectsSystem(system, "OpenDIAG config file not found: " + system);
    rejectsSystem(temporary, "OpenDIAG config file not found: " + temporary);
    rejectsSystem("", "Cannot locate the system configuration folder");
    rejectsOverride(overridePath);
    std::ofstream(system.c_str()) << "[device]\nName=System\n[logging]\nEnabled=1\n";
    check(platform::selectConfigPath("", system) == system,
          "Absent override selects the existing system configuration");
    rejectsOverride(overridePath);
    std::ofstream(overridePath.c_str()) << "[device]\nName=Override\n";
    check(platform::selectConfigPath(overridePath, system) == overridePath,
          "Existing explicit configuration precedes system configuration");
    rejectsOverride(temporary);
    const char *original = getenv("OPENDIAG_DRIVER_CONFIG");
    const bool hadOverride = original != nullptr;
    const std::string saved = original ? original : "";
    setenv("OPENDIAG_DRIVER_CONFIG", overridePath.c_str(), 1);
    check(platform::configPath() == overridePath,
          "Configuration resolver honors the environment override");
    check(platform::readSetting(platform::configPath(), "device", "Name", "") == "Override",
          "Settings come from the selected file");
    check(platform::readNumber(platform::configPath(), "logging", "Enabled", 0) == 0,
          "Selected settings are not merged with the system file");
    char originalDirectory[PATH_MAX];
    check(getcwd(originalDirectory, sizeof(originalDirectory)) && !chdir(temporary.c_str()),
          "Enter configuration folder for relative-path check");
    rejectsOverride("missing.ini");
    setenv("OPENDIAG_DRIVER_CONFIG", "override.ini", 1);
    check(platform::configPath() == overridePath,
          "Relative override is resolved before reading the INI file");
    check(!chdir(originalDirectory), "Restore working directory");
    setenv("OPENDIAG_DRIVER_CONFIG", overridePath.c_str(), 1);
    unlink(overridePath.c_str());
    check(platform::configWritePath() == overridePath,
          "Explicit write path can create a new configuration");
    bool rejected = false;
    try {
        platform::configPath();
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    check(rejected, "Configuration resolver rejects a missing environment override");
    setenv("OPENDIAG_DRIVER_CONFIG", "", 1);
    check(platform::configWritePath() == platform::systemConfigPath(),
          "Empty override uses the default write path");
    unsetenv("OPENDIAG_DRIVER_CONFIG");
    check(platform::configWritePath() == platform::systemConfigPath(),
          "Default writes target the system configuration");
    if (hadOverride) {
        setenv("OPENDIAG_DRIVER_CONFIG", saved.c_str(), 1);
    }
    const std::string logs = platform::logDirectory(directory);
#ifdef __APPLE__
    check(platform::systemConfigPath() == "/Library/Application Support/OpenDIAG/opendiag.ini",
          "macOS system configuration location");
    const std::string expectedLogs = std::string(getenv("HOME")) + "/Library/Logs/OpenDIAG/";
#else
    check(platform::systemConfigPath() == "/etc/opendiag/opendiag.ini",
          "Linux system configuration location");
    const char *state = getenv("XDG_STATE_HOME");
    const std::string expectedLogs = state && state[0] == '/'
                                         ? std::string(state) + "/opendiag/"
                                         : std::string(getenv("HOME")) + "/.local/state/opendiag/";
#endif
    check(logs == expectedLogs && logs != directory, "Logs use a writable per-user folder");
    // The home folder is only trusted when it is absolute; otherwise the password database
    // answers, and a relative XDG_STATE_HOME is ignored.
    const char *home = getenv("HOME");
    const bool hadHome = home != nullptr;
    const std::string savedHome = home ? home : "";
    const char *xdgState = getenv("XDG_STATE_HOME");
    const bool hadState = xdgState != nullptr;
    const std::string savedState = xdgState ? xdgState : "";
#ifdef __APPLE__
    const std::string homeLogs = "/Library/Logs/OpenDIAG/";
#else
    const std::string homeLogs = "/.local/state/opendiag/";
    setenv("XDG_STATE_HOME", "relative-state", 1);
    check(platform::userLogDirectory() == savedHome + homeLogs,
          "Relative XDG_STATE_HOME falls back to the home folder");
    unsetenv("XDG_STATE_HOME");
#endif
    setenv("HOME", "relative-home", 1);
    const passwd *entry = getpwuid(getuid());
    check(entry && entry->pw_dir && entry->pw_dir[0] == '/' &&
              platform::userLogDirectory() == std::string(entry->pw_dir) + homeLogs,
          "Relative HOME is replaced by the password database home");
    if (hadHome) {
        setenv("HOME", savedHome.c_str(), 1);
    } else {
        unsetenv("HOME");
    }
    if (hadState) {
        setenv("XDG_STATE_HOME", savedState.c_str(), 1);
    }
    // A regular file in the directory chain prevents creation of the log folder.
    const char *variable;
    std::string blocked;
#ifdef __APPLE__
    variable = "HOME";
    blocked = system;
#else
    variable = "XDG_STATE_HOME";
    blocked = system;
#endif
    original = getenv(variable);
    const bool hadLogPath = original != nullptr;
    const std::string savedLogPath = original ? original : "";
    setenv(variable, blocked.c_str(), 1);
    check(platform::logDirectory(directory) == directory,
          "Unusable user log folder falls back to the library folder");
    if (hadLogPath) {
        setenv(variable, savedLogPath.c_str(), 1);
    } else {
        unsetenv(variable);
    }
    unlink(system.c_str());
    rmdir(temporary.c_str());
}

static void testClosedStream() {
    platform::Stream stream;
    const unsigned char byte = 0x5a;
    bool expired = false;
    try {
        stream.write(&byte, 1, platform::milliseconds());
    } catch (const std::runtime_error &) {
        expired = true;
    }
    check(expired, "An expired transport deadline is reported");
    bool closed = false;
    try {
        stream.write(&byte, 1, platform::milliseconds() + 1000);
    } catch (const std::runtime_error &) {
        closed = true;
    }
    check(closed, "Writing to a closed transport is reported");
}

static void testSerialConfigurationFailure() {
    // Descriptor -1 is never open, so tcsetattr fails and the failure must reach the caller.
    termios settings = {};
    bool rejected = false;
    try {
        platform::configureSerial(-1, settings, 115200);
    } catch (const std::runtime_error &error) {
        rejected = !strcmp(error.what(), "Serial configuration failed");
    }
    check(rejected, "A serial device that cannot be configured is reported");
}

static void testRemovedWorkingDirectory() {
    // With the working directory removed, getcwd fails and a relative path cannot be made absolute.
    const std::string directory = platform::moduleDirectory(&moduleAnchor);
    const std::string temporary = directory + "cwd-test-" + std::to_string(getpid()) + "/";
    check(platform::createDirectories(temporary), "Create working directory test folder");
    const int original = open(".", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    check(original >= 0, "Open the test working directory");
    check(!chdir(temporary.c_str()), "Enter the working directory test folder");
    check(!rmdir(temporary.c_str()), "Remove the working directory under the test");
    bool rejected = false;
    try {
        platform::absolutePath("config.ini");
    } catch (const std::runtime_error &error) {
        rejected = !strcmp(error.what(), "Cannot resolve the configuration path");
    }
    const bool restored = !fchdir(original);
    ::close(original);
    check(restored, "Return to the test working directory");
    check(rejected, "A relative path without a working directory is reported");
}

static std::string readWholeFile(const std::string &path) {
    std::ifstream file(path.c_str());
    return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

static void testIniFiles() {
    const std::string directory = platform::moduleDirectory(&moduleAnchor);
    const std::string temporary = directory + "ini-test-" + std::to_string(getpid()) + "/";
    check(platform::createDirectories(temporary), "Create INI test folder");
    const std::string path = temporary + "settings.ini";
    std::ofstream(path.c_str()) << "; a comment\n# another comment\n\n[device]\n"
                                << "Quoted='quoted value'\n"
                                << "Double = \"double\"\n"
                                << "Unclosed='open\n";
    check(platform::readSetting(path, "device", "Quoted", "") == "quoted value",
          "Single-quoted INI values are unwrapped");
    check(platform::readSetting(path, "device", "Double", "") == "double",
          "Double-quoted INI values are unwrapped");
    check(platform::readSetting(path, "device", "Unclosed", "") == "'open",
          "An unmatched quote stays in the INI value");
    check(platform::readSetting(path, "device", "Missing", "fallback") == "fallback",
          "A missing INI key returns the fallback");
    std::ofstream(path.c_str()) << "[device]\nLong=" << std::string(255, 'x') << "\n";
    bool rejected = false;
    try {
        platform::readSetting(path, "device", "Long", "");
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    check(rejected, "Overlong INI values are rejected");

    // Only a regular file is logged to. A full log that cannot be rotated is left as it is.
    platform::appendLog("/dev/null", "ignored\n", 1);
    const std::string log = temporary + "log.txt";
    std::ofstream(log.c_str()) << "0123456789";
    check(mkdir((log + ".1").c_str(), 0700) == 0, "Block log rotation with a folder");
    platform::appendLog(log.c_str(), "next\n", 4);
    check(readWholeFile(log) == "0123456789", "A log that cannot rotate is not written");
    rmdir((log + ".1").c_str());
    unlink(log.c_str());
    unlink(path.c_str());
    rmdir(temporary.c_str());
}

static void testStreamFailures() {
    const std::string directory = platform::moduleDirectory(&moduleAnchor);
    const std::string temporary = directory + "stream-test-" + std::to_string(getpid()) + "/";
    check(platform::createDirectories(temporary), "Create stream test folder");
    const std::string regular = temporary + "not-a-serial-port";
    std::ofstream(regular.c_str()) << "x";
    bool notSerial = false;
    try {
        platform::Stream stream;
        stream.openCom(regular, 115200);
    } catch (const std::runtime_error &error) {
        notSerial = !strcmp(error.what(), "Cannot configure the serial device");
    }
    check(notSerial, "A regular file is refused as a serial device");
    unlink(regular.c_str());
    rmdir(temporary.c_str());

    bool hostRejected = false;
    try {
        platform::Stream stream;
        stream.openTcp("localhost", 1, 100);
    } catch (const std::runtime_error &error) {
        hostRejected = !strcmp(error.what(), "Host must be an IPv4 address");
    }
    check(hostRejected, "A TCP host must be an IPv4 address");

    // Port 1 has no listener on a test host, so the connection is refused.
    bool refused = false;
    try {
        platform::Stream stream;
        stream.openTcp("127.0.0.1", 1, 1000);
    } catch (const std::runtime_error &error) {
        refused = !strcmp(error.what(), "TCP connect failed") ||
                  !strcmp(error.what(), "TCP connection failed");
    }
    check(refused, "A TCP connection to a closed port is reported");
}

int main() {
    try {
        testSerial();
        testSerialConfigurationFailure();
        testStreamFailures();
        testRemovedWorkingDirectory();
        testLease();
        testPaths();
        testIniFiles();
        testClosedStream();
        testLogging();
        printf("PASS %u platform checks: serial, deadlines, ownership, paths and logging\n",
               checks);
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
