// SPDX-License-Identifier: GPL-3.0-only
#include "../config/port_classify.h"
#include <cstdio>
#include <stdexcept>

static unsigned checks;

static void check(bool condition, const char *message) {
    ++checks;
    if (!condition) {
        throw std::runtime_error(message);
    }
}

static void testDeviceRoles() {
    const std::string instance = "6&1f17fa39&0&0000";
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_00\\" + instance) == PORT_DATA,
          "MI_00 is the data port");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_02\\" + instance) == PORT_DEBUG,
          "MI_02 is the debug console");
    check(classifyDevicePath("VID_1209&PID_D1A6&MI_00\\" + instance) == PORT_DATA,
          "Enum key form without the USB prefix");
    check(classifyDevicePath("usb\\vid_1209&pid_d1a6&mi_02\\" + instance) == PORT_DEBUG,
          "Hardware id is case-insensitive");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_00\\" + std::string(200, 'x')) == PORT_DATA,
          "Longest accepted instance id");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_01\\" + instance) == PORT_OTHER,
          "Other OpenDIAG interfaces are not labelled");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6\\" + instance) == PORT_OTHER,
          "Composite parent without an interface is other");
    check(classifyDevicePath("USB\\VID_0403&PID_D1A6&MI_00\\" + instance) == PORT_OTHER,
          "Other vendor is other");
    check(classifyDevicePath("USB\\VID_1209&PID_0001&MI_00\\" + instance) == PORT_OTHER,
          "Other product is other");
}

static void testMalformedPaths() {
    const std::string hardware = "VID_1209&PID_D1A6&MI_00";
    check(classifyDevicePath("") == PORT_OTHER, "Empty path is other");
    check(classifyDevicePath("USB\\") == PORT_OTHER, "USB prefix alone is other");
    check(classifyDevicePath("USB\\" + hardware) == PORT_OTHER,
          "Hardware id without an instance is other");
    check(classifyDevicePath("USB\\" + hardware + "\\") == PORT_OTHER,
          "Empty instance id is other");
    check(classifyDevicePath("USB\\" + hardware + "\\6 1f") == PORT_OTHER,
          "Instance id with a space is other");
    check(classifyDevicePath("USB\\" + hardware + std::string("\\6\x01") ) == PORT_OTHER,
          "Instance id with a control character is other");
    check(classifyDevicePath("USB\\" + hardware + "\\" + std::string(1, '\x7f')) == PORT_OTHER,
          "Instance id with a non-ASCII byte is other");
    check(classifyDevicePath("USB\\" + hardware + "\\" + std::string(201, 'x')) == PORT_OTHER,
          "Instance id longer than 200 characters is other");
    check(classifyDevicePath("USB\\" + hardware + "\\a\\b") == PORT_OTHER,
          "Instance id containing a backslash is other");
    check(classifyDevicePath("USB\\VIX_1209&PID_D1A6&MI_00\\6") == PORT_OTHER,
          "Wrong vendor label is other");
    check(classifyDevicePath("USB\\VID_1209&PIX_D1A6&MI_00\\6") == PORT_OTHER,
          "Wrong product label is other");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MX_00\\6") == PORT_OTHER,
          "Wrong interface label is other");
    check(classifyDevicePath("USB\\VID_12G9&PID_D1A6&MI_00\\6") == PORT_OTHER,
          "Non-hex vendor is other");
    check(classifyDevicePath("USB\\VID_1209&PID_D1G6&MI_00\\6") == PORT_OTHER,
          "Non-hex product is other");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_0G\\6") == PORT_OTHER,
          "Non-hex interface is other");
    check(classifyDevicePath("USB\\VID_1209&PID_D1A6&MI_0\\6") == PORT_OTHER,
          "Short interface number is other");
}

static void testPortNames() {
    std::string com = "unchanged";
    check(parsePortName("COM10", com) && com == "COM10", "Parse a two-digit COM port");
    check(parsePortName("com9", com) && com == "COM9", "Parse a lower-case PortName");
    check(parsePortName("COM65535", com) && com == "COM65535", "Parse the largest COM number");
    check(!parsePortName("", com), "Empty PortName is rejected");
    check(!parsePortName("COM", com), "COM without digits is rejected");
    check(!parsePortName("CO1", com), "Two letters are rejected");
    check(!parsePortName("LPT1", com), "Non-COM port is rejected");
    check(!parsePortName("COMX1", com), "Non-digit after COM is rejected");
    check(!parsePortName("COM1a", com), "Trailing letter is rejected");
    check(!parsePortName("COM10 ", com), "Trailing space is rejected");
    check(!parsePortName("COM0", com), "COM0 is rejected");
    check(!parsePortName("COM01", com), "Leading zero is rejected");
    check(!parsePortName("COM123456", com), "Six digits are rejected");
    check(com == "COM65535", "Rejected names leave the output unchanged");
}

static void testLabels() {
    check(portLabel("COM10", PORT_DATA) == "COM10 (data port)", "Data port label");
    check(portLabel("COM9", PORT_DEBUG) == "COM9 (debug console)", "Debug console label");
    check(portLabel("COM3", PORT_OTHER) == "COM3", "Other ports keep the bare name");
    check(comFromLabel("COM10 (data port)") == "COM10", "Strip the data label");
    check(comFromLabel("COM9 (debug console)") == "COM9", "Strip the debug label");
    check(comFromLabel("COM3") == "COM3", "Bare name is unchanged");
    check(comFromLabel(" (data port)") == " (data port)", "Suffix alone is unchanged");
    check(comFromLabel("") == "", "Empty text is unchanged");
    check(comFromLabel("COM3 (other)") == "COM3 (other)", "Unknown suffix is unchanged");
    check(comFromLabel(portLabel("COM4", PORT_DEBUG)) == "COM4", "Label round trip");
}

int main() {
    try {
        testDeviceRoles();
        testMalformedPaths();
        testPortNames();
        testLabels();
        printf("PASS %u port classifier checks: device roles, PortName and labels\n", checks);
        return 0;
    } catch (const std::exception &error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
