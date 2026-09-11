// SPDX-License-Identifier: GPL-3.0-only
#include "settings.h"
#include <algorithm>
#include <stdexcept>
#include <stdio.h>

std::string applicationDirectory() {
    char path[MAX_PATH];
    DWORD length = GetModuleFileNameA(NULL, path, sizeof(path));
    if (!length || length >= sizeof(path)) {
        throw std::runtime_error("The application path is too long.");
    }
    char *filename = strrchr(path, '\\');
    if (!filename) {
        throw std::runtime_error("Cannot locate the application folder.");
    }
    filename[1] = 0;
    return path;
}

static std::string readValue(const std::string &path, const char *key, const char *fallback) {
    char text[256];
    DWORD length =
        GetPrivateProfileStringA("device", key, fallback, text, sizeof(text), path.c_str());
    if (length >= sizeof(text) - 1) {
        throw std::runtime_error(std::string("Setting is too long: ") + key);
    }
    return text;
}

AdapterSettings readSettings(const std::string &path) {
    AdapterSettings settings;
    settings.name = readValue(path, "Name", "OpenDIAG");
    settings.transport = readValue(path, "Transport", "com");
    settings.com = readValue(path, "COM", "COM3");
    settings.host = readValue(path, "Host", "192.168.56.1");
    settings.port = GetPrivateProfileIntA("device", "Port", 23200, path.c_str());
    settings.baud = GetPrivateProfileIntA("device", "Baud", 115200, path.c_str());
    settings.timeout = GetPrivateProfileIntA("device", "RpcTimeout", 8000, path.c_str());
    settings.logging = GetPrivateProfileIntA("logging", "Enabled", 0, path.c_str()) != 0;
    settings.logSizeKB = GetPrivateProfileIntA("logging", "MaxFileKB", 4096, path.c_str());
    return settings;
}

unsigned parseNumber(const std::string &text,
                     const char *label,
                     unsigned minimum,
                     unsigned maximum) {
    unsigned result = 0;
    bool valid = !text.empty();
    for (size_t i = 0; valid && i < text.size(); ++i) {
        unsigned digit = (unsigned)(text[i] - '0');
        if (digit > 9 || result > maximum / 10 ||
            (result == maximum / 10 && digit > maximum % 10)) {
            valid = false;
        } else {
            result = result * 10 + digit;
        }
    }
    if (!valid || result < minimum || result > maximum) {
        char message[160];
        platform::format(message,
                         sizeof(message),
                         "%s must be a whole number from %u to %u.",
                         label,
                         minimum,
                         maximum);
        throw std::runtime_error(message);
    }
    return result;
}

static void validateCom(const std::string &name, const char *label) {
    if (name.size() < 4 || _strnicmp(name.c_str(), "COM", 3) != 0) {
        throw std::runtime_error(std::string(label) + " must be a COM port, for example COM3.");
    }
    parseNumber(name.substr(3), label, 1, 65535);
}

static void validateHost(const std::string &host, const char *label) {
    size_t start = 0;
    for (unsigned part = 0; part < 4; ++part) {
        size_t end = host.find('.', start);
        if ((part < 3 && end == std::string::npos) || (part == 3 && end != std::string::npos)) {
            throw std::runtime_error(std::string(label) + " must be an IPv4 address.");
        }
        std::string field = host.substr(start, end - start);
        if (field.size() > 1 && field[0] == '0') {
            throw std::runtime_error(std::string(label) + " must not contain leading zeros.");
        }
        parseNumber(field, label, 0, 255);
        start = end + 1;
    }
    if (host == "0.0.0.0" || host == "255.255.255.255") {
        throw std::runtime_error(std::string(label) + " must identify a host.");
    }
}

void validateSettings(const AdapterSettings &settings) {
    if (settings.name.empty() || settings.name.size() > 79) {
        throw std::runtime_error("Adapter name must contain 1 to 79 characters.");
    }
    for (size_t i = 0; i < settings.name.size(); ++i) {
        unsigned char c = (unsigned char)settings.name[i];
        if (c < 32 || c > 126) {
            throw std::runtime_error("Adapter name must use printable ASCII characters.");
        }
    }
    if (settings.port < 1 || settings.port > 65535) {
        throw std::runtime_error("TCP ports must be from 1 to 65535.");
    }
    if (settings.timeout < 100 || settings.timeout > 30000) {
        throw std::runtime_error("Connection timeout must be from 100 to 30000 ms.");
    }
    if (settings.baud < 300 || settings.baud > 4000000) {
        throw std::runtime_error("Serial baud rate must be from 300 to 4000000.");
    }
    if (settings.logSizeKB < 64 || settings.logSizeKB > 65536) {
        throw std::runtime_error("Log file limit must be from 64 to 65536 KB.");
    }
    if (settings.transport == "com") {
        validateCom(settings.com, "COM port");
    } else if (settings.transport == "tcp") {
        validateHost(settings.host, "Host");
    } else {
        throw std::runtime_error("Select USB serial or TCP/IP.");
    }
}

static void writeValue(const std::string &path,
                       const char *section,
                       const char *key,
                       const std::string &value) {
    if (!WritePrivateProfileStringA(section, key, value.c_str(), path.c_str())) {
        throw std::runtime_error(std::string("Cannot save setting: ") + key);
    }
}

static void writeNumber(const std::string &path,
                        const char *section,
                        const char *key,
                        unsigned value) {
    char text[24];
    platform::format(text, sizeof(text), "%u", value);
    writeValue(path, section, key, text);
}

void saveSettings(const std::string &path, const AdapterSettings &settings) {
    validateSettings(settings);
    size_t slash = path.find_last_of("\\/");
    if (slash == std::string::npos) {
        throw std::runtime_error("Configuration path must be absolute.");
    }
    DWORD attributes = GetFileAttributesA(path.c_str());
    if (attributes != INVALID_FILE_ATTRIBUTES &&
        (attributes & (FILE_ATTRIBUTE_READONLY | FILE_ATTRIBUTE_DIRECTORY))) {
        throw std::runtime_error("The configuration file is read-only or is a directory.");
    }
    char temporary[MAX_PATH];
    if (!GetTempFileNameA(path.substr(0, slash + 1).c_str(), "odg", 0, temporary)) {
        throw std::runtime_error("Cannot write to the installation folder. Check its permissions.");
    }
    try {
        if (attributes != INVALID_FILE_ATTRIBUTES && !CopyFileA(path.c_str(), temporary, FALSE)) {
            throw std::runtime_error("Cannot copy the existing configuration.");
        }
        writeValue(temporary, "device", "Name", settings.name);
        writeValue(temporary, "device", "Transport", settings.transport);
        writeValue(temporary, "device", "COM", settings.com);
        writeValue(temporary, "device", "Host", settings.host);
        writeNumber(temporary, "device", "Port", settings.port);
        writeNumber(temporary, "device", "Baud", settings.baud);
        writeNumber(temporary, "device", "RpcTimeout", settings.timeout);
        const char *obsoleteKeys[] = {"DebugCOM", "DebugHost", "DebugPort", "AutoMode"};
        for (unsigned i = 0; i < sizeof(obsoleteKeys) / sizeof(obsoleteKeys[0]); ++i) {
            if (!WritePrivateProfileStringA("device", obsoleteKeys[i], NULL, temporary)) {
                throw std::runtime_error("Cannot remove obsolete connection settings.");
            }
        }
        writeNumber(temporary, "logging", "Enabled", settings.logging);
        writeNumber(temporary, "logging", "MaxFileKB", settings.logSizeKB);
        WritePrivateProfileStringA(NULL, NULL, NULL, temporary);
        if (!MoveFileExA(temporary,
                         path.c_str(),
                         MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
            throw std::runtime_error("Cannot replace opendiag.ini. Check its permissions.");
        }
        WritePrivateProfileStringA(NULL, NULL, NULL, path.c_str());
    } catch (...) {
        DeleteFileA(temporary);
        throw;
    }
}

std::vector<std::string> availableComPorts() {
    std::vector<std::string> ports;
    HKEY key;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) !=
        ERROR_SUCCESS) {
        return ports;
    }
    for (DWORD index = 0;; ++index) {
        char name[256];
        char value[256];
        DWORD nameSize = sizeof(name);
        DWORD valueSize = sizeof(value);
        DWORD type = 0;
        LONG status =
            RegEnumValueA(key, index, name, &nameSize, NULL, &type, (BYTE *)value, &valueSize);
        if (status == ERROR_NO_MORE_ITEMS) {
            break;
        }
        if (status == ERROR_SUCCESS && type == REG_SZ && valueSize > 0 &&
            valueSize < sizeof(value)) {
            value[valueSize] = 0;
            ports.push_back(value);
        }
    }
    RegCloseKey(key);
    std::sort(ports.begin(), ports.end());
    ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    return ports;
}
