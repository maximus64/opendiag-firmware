// SPDX-License-Identifier: GPL-3.0-only
#include "port_classify.h"

static const char USB_PREFIX[] = "USB\\";
static const char DATA_LABEL[] = " (data port)";
static const char DEBUG_LABEL[] = " (debug console)";
static const unsigned OPENDIAG_VID = 0x1209;
static const unsigned OPENDIAG_PID = 0xD1A6;
static const size_t HARDWARE_ID_SIZE = 23; // "VID_1209&PID_D1A6&MI_00"
static const size_t MAX_INSTANCE_SIZE = 200;
static const size_t MAX_COM_DIGITS = 5;

static char upper(char c) {
    return c >= 'a' && c <= 'z' ? (char)(c - 'a' + 'A') : c;
}

// Case-insensitive match of word at position at; false if the text is too short.
static bool matchesAt(const std::string &text, size_t at, const char *word) {
    for (size_t i = 0; word[i]; ++i) {
        if (at + i >= text.size() || upper(text[at + i]) != upper(word[i])) {
            return false;
        }
    }
    return true;
}

// The caller guarantees that text holds digits characters from at.
static bool hexAt(const std::string &text, size_t at, size_t digits, unsigned &value) {
    value = 0;
    for (size_t i = 0; i < digits; ++i) {
        char c = upper(text[at + i]);
        unsigned digit = 0;
        if (c >= '0' && c <= '9') {
            digit = (unsigned)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            digit = (unsigned)(c - 'A' + 10);
        } else {
            return false;
        }
        value = value * 16 + digit;
    }
    return true;
}

// Instance ids are printable ASCII without spaces or backslashes.
static bool validInstance(const std::string &instance) {
    if (instance.empty() || instance.size() > MAX_INSTANCE_SIZE) {
        return false;
    }
    for (size_t i = 0; i < instance.size(); ++i) {
        unsigned char c = (unsigned char)instance[i];
        if (c <= 0x20 || c > 0x7e || c == '\\') {
            return false;
        }
    }
    return true;
}

// Parses "VID_1209&PID_D1A6&MI_00" in any letter case.
static bool parseHardwareId(const std::string &hardwareId,
                            unsigned &vendor,
                            unsigned &product,
                            unsigned &interface) {
    if (hardwareId.size() != HARDWARE_ID_SIZE || !matchesAt(hardwareId, 0, "VID_") ||
        !matchesAt(hardwareId, 8, "&PID_") || !matchesAt(hardwareId, 17, "&MI_")) {
        return false;
    }
    return hexAt(hardwareId, 4, 4, vendor) && hexAt(hardwareId, 13, 4, product) &&
           hexAt(hardwareId, 21, 2, interface);
}

PortRole classifyDevicePath(const std::string &path) {
    std::string text = path;
    if (matchesAt(text, 0, USB_PREFIX)) {
        text.erase(0, sizeof(USB_PREFIX) - 1);
    }
    size_t split = text.find('\\');
    if (split == std::string::npos || !validInstance(text.substr(split + 1))) {
        return PORT_OTHER;
    }
    unsigned vendor = 0;
    unsigned product = 0;
    unsigned interface = 0;
    if (!parseHardwareId(text.substr(0, split), vendor, product, interface) ||
        vendor != OPENDIAG_VID || product != OPENDIAG_PID) {
        return PORT_OTHER;
    }
    if (interface == 0) {
        return PORT_DATA;
    }
    if (interface == 2) {
        return PORT_DEBUG;
    }
    return PORT_OTHER;
}

bool parsePortName(const std::string &value, std::string &com) {
    const size_t prefix = 3;
    if (value.size() <= prefix || value.size() > prefix + MAX_COM_DIGITS ||
        !matchesAt(value, 0, "COM") || value[prefix] == '0') {
        return false;
    }
    for (size_t i = prefix; i < value.size(); ++i) {
        if (value[i] < '0' || value[i] > '9') {
            return false;
        }
    }
    com = "COM" + value.substr(prefix);
    return true;
}

std::string portLabel(const std::string &com, PortRole role) {
    if (role == PORT_DATA) {
        return com + DATA_LABEL;
    }
    if (role == PORT_DEBUG) {
        return com + DEBUG_LABEL;
    }
    return com;
}

std::string comFromLabel(const std::string &text) {
    const std::string labels[] = {DATA_LABEL, DEBUG_LABEL};
    for (size_t i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i) {
        const std::string &label = labels[i];
        if (text.size() > label.size() &&
            text.compare(text.size() - label.size(), label.size(), label) == 0) {
            return text.substr(0, text.size() - label.size());
        }
    }
    return text;
}
