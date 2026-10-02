// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <termios.h>

// What posix.cpp needs from the host it is built for: linux.cpp or darwin.cpp.
namespace platform {
// A non-blocking, close-on-exec TCP socket, or -1.
int openTcpSocket();
// Applies raw settings to an open serial device at the requested baud rate.
void configureSerial(int file, termios &settings, unsigned baud);
} // namespace platform
