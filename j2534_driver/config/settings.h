// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../platform/platform.h"
#include <string>
#include <vector>
#include <windows.h>

struct AdapterSettings {
    std::string name;
    std::string transport;
    std::string com;
    std::string host;
    unsigned port;
    unsigned baud;
    unsigned timeout;
    bool logging;
    unsigned logSizeKB;
};

std::string applicationDirectory();
AdapterSettings readSettings(const std::string &path);
void validateSettings(const AdapterSettings &settings);
void saveSettings(const std::string &path, const AdapterSettings &settings);
unsigned parseNumber(const std::string &text,
                     const char *label,
                     unsigned minimum,
                     unsigned maximum);
std::vector<std::string> availableComPorts();
std::string checkConnection(const std::string &directory, const AdapterSettings &settings);
std::string driverVersions(const std::string &directory);
