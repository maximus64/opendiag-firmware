// SPDX-License-Identifier: GPL-3.0-only
#include "settings.h"
#include <stdexcept>
#include <stdio.h>

static std::string fileVersion(const std::string &path) {
    DWORD ignored = 0;
    DWORD size = GetFileVersionInfoSizeA(path.c_str(), &ignored);
    if (!size) {
        return "not installed";
    }
    std::vector<unsigned char> data(size);
    if (!GetFileVersionInfoA(path.c_str(), 0, size, &data[0])) {
        return "unavailable";
    }
    VS_FIXEDFILEINFO *info = NULL;
    UINT length = 0;
    if (!VerQueryValueA(&data[0], "\\", (void **)&info, &length) || length < sizeof(*info)) {
        return "unavailable";
    }
    char version[64];
    platform::format(version,
                     sizeof(version),
                     "%u.%u.%u.%u",
                     HIWORD(info->dwFileVersionMS),
                     LOWORD(info->dwFileVersionMS),
                     HIWORD(info->dwFileVersionLS),
                     LOWORD(info->dwFileVersionLS));
    return version;
}

std::string driverVersions(const std::string &directory) {
    return "04.04 DLL: " + fileVersion(directory + "odg40432.dll") +
           "    05.00 DLL: " + fileVersion(directory + "opendiag32.dll");
}

class DeviceCheck {
    typedef long(WINAPI *OpenFunction)(const char *, unsigned long *);
    typedef long(WINAPI *CloseFunction)(unsigned long);
    typedef long(WINAPI *VersionFunction)(unsigned long, char *, char *, char *);
    typedef long(WINAPI *ErrorFunction)(char *);
    HMODULE module_;
    unsigned long device_;
    bool opened_;
    OpenFunction open_;
    CloseFunction close_;
    VersionFunction version_;
    ErrorFunction error_;

    void *function(const char *name) {
        FARPROC address = GetProcAddress(module_, name);
        if (!address) {
            throw std::runtime_error(std::string("Missing DLL entry point: ") + name);
        }
        return reinterpret_cast<void *>(address);
    }

    void check(long status, const char *operation) {
        if (status == 0) {
            return;
        }
        char detail[80] = {};
        error_(detail);
        detail[sizeof(detail) - 1] = 0;
        char message[256];
        platform::format(message,
                         sizeof(message),
                         "%s failed (0x%08lX). %s",
                         operation,
                         status,
                         detail);
        if (status == 0x0E) {
            throw std::runtime_error(
                "The adapter is in use. Close the diagnostic application and try again.");
        }
        throw std::runtime_error(message);
    }

  public:
    explicit DeviceCheck(const std::string &path)
        : module_(NULL),
          device_(0),
          opened_(false),
          open_(NULL),
          close_(NULL),
          version_(NULL),
          error_(NULL) {
        module_ = LoadLibraryA(path.c_str());
        if (!module_) {
            throw std::runtime_error("Cannot load opendiag32.dll. Install it beside this program.");
        }
        try {
            open_ = (OpenFunction)function("PassThruOpen");
            close_ = (CloseFunction)function("PassThruClose");
            version_ = (VersionFunction)function("PassThruReadVersion");
            error_ = (ErrorFunction)function("PassThruGetLastError");
        } catch (...) {
            FreeLibrary(module_);
            throw;
        }
    }

    ~DeviceCheck() {
        if (opened_) {
            close_(device_);
        }
        FreeLibrary(module_);
    }

    std::string run(const AdapterSettings &settings) {
        std::string name = "J2534-1:" + settings.name;
        check(open_(name.c_str(), &device_), "Open adapter");
        opened_ = true;
        char firmware[80] = {};
        char dll[80] = {};
        char api[80] = {};
        check(version_(device_, firmware, dll, api), "Read version");
        firmware[79] = 0;
        dll[79] = 0;
        api[79] = 0;
        long status = close_(device_);
        opened_ = false;
        check(status, "Close adapter");
        return std::string("Connected to ") + settings.name + ".\r\nFirmware: " + firmware +
               "\r\n" + dll + " / API " + api;
    }

  private:
    DeviceCheck(const DeviceCheck &);
    DeviceCheck &operator=(const DeviceCheck &);
};

std::string checkConnection(const std::string &directory, const AdapterSettings &settings) {
    DeviceCheck check(directory + "opendiag32.dll");
    return check.run(settings);
}
