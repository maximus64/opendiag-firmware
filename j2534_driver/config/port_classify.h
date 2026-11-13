// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <string>

// Which USB CDC function of an OpenDIAG board a COM port is. This header and
// port_classify.cpp have no Windows includes, so the rules are tested on the host.
enum PortRole {
    PORT_OTHER,
    PORT_DATA,
    PORT_DEBUG
};

// Classifies a PnP device path, "USB\VID_1209&PID_D1A6&MI_00\6&1f17fa39&0&0000",
// or the same path without the "USB\" prefix (the Enum\USB registry key form).
// MI_00 is the data port and MI_02 the debug console. Anything else, including a
// missing or malformed instance id, is PORT_OTHER.
PortRole classifyDevicePath(const std::string &path);

// Reads a PortName registry value such as "COM10" in any letter case. Returns true
// and the canonical name in com only for COM followed by 1 to 5 digits with no
// leading zero. com is left unchanged otherwise.
bool parsePortName(const std::string &value, std::string &com);

// Dropdown text: "COM10 (data port)", "COM9 (debug console)", or the bare name
// for PORT_OTHER.
std::string portLabel(const std::string &com, PortRole role);

// Removes the suffix that portLabel adds, so a selected label gives the COM name.
// Text without one of those suffixes is returned unchanged.
std::string comFromLabel(const std::string &text);
