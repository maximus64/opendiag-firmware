#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""OpenDIAG miniterm: raw output, CR on Enter, and console highlighting."""

import argparse
from functools import partial
import math
import os
import re
import sys
import threading
import time

try:
    import serial
    from serial.tools import miniterm
    from serial.urlhandler.protocol_socket import Serial as SocketSerial
except ImportError:
    raise SystemExit("pySerial is required: python -m pip install pyserial")


RESET = b"\x1b[0m"
RED = b"\x1b[31m"
GREEN = b"\x1b[32m"
YELLOW = b"\x1b[33m"
CYAN = b"\x1b[36m"
MAGENTA = b"\x1b[35m"
DIM = b"\x1b[90m"

LOG = re.compile(rb"^([EWIDV]) (?:\(|[^\s:]+:)")
LOG_COLORS = {b"E": RED, b"W": YELLOW, b"I": GREEN, b"D": DIM, b"V": DIM}
RULES = [
    (re.compile(rb"^(?:ERROR\b|ERR\b|BUS ERROR|CAN ERROR|DATA ERROR|"
                rb"UNABLE TO CONNECT|BUFFER FULL|LV RESET|\?|"
                rb"Guru Meditation|Backtrace:|assert failed|abort\(\)|"
                rb"(?:send|receive|init) failed|bad |not hex:|hex string too long|"
                rb"line too long)", re.I), RED),
    (re.compile(rb"^(?:NO DATA|STOPPED|SEARCHING|BUS BUSY|Usage:|note:)", re.I), YELLOW),
    (re.compile(rb"^OK\b"), GREEN),
    (re.compile(rb"^rx ", re.I), CYAN),
    (re.compile(rb"^tx ", re.I), MAGENTA),
    (re.compile(rb"^>"), CYAN),
    (re.compile(rb"^(?:[0-9A-F]{2} ){2}|^[tTrR][0-9A-F]{3,8}[0-8]", re.I), CYAN),
]


class Highlighter:
    """Color a byte stream without waiting for newline-terminated prompts."""

    def __init__(self):
        self.prefix = b""
        self.color = b""
        self.passthrough = False

    def reset_color(self):
        reset = RESET if self.color else b""
        self.color = b""
        return reset

    def feed(self, data):
        output = []
        for part in re.split(rb"([\r\n])", data):
            if not part:
                continue
            if part in (b"\r", b"\n"):
                output.extend((self.reset_color(), part))
                self.prefix = b""
                self.passthrough = False
                continue

            # Leave device ANSI sequences and terminal editing intact, even
            # when an escape sequence is split between serial reads.
            if b"\x1b" in part or b"\b" in part:
                output.append(self.reset_color())
                self.passthrough = True
            self.prefix += part[:max(0, 256 - len(self.prefix))]
            if not self.passthrough and not self.color:
                prefix = self.prefix.lstrip(b" \t")
                log = LOG.match(prefix)
                if log:
                    self.color = LOG_COLORS[log[1]]
                else:
                    self.color = next((color for pattern, color in RULES
                                       if pattern.match(prefix)), b"")
                output.append(self.color)
            output.append(part)
        return b"".join(output)


class ReconnectingSocket(SocketSerial):
    """Keep miniterm alive while its reader retries a dropped TCP connection."""

    def __init__(self, *args, retry_delay=5, **kwargs):
        self.retry_delay = retry_delay
        self.cancelled = threading.Event()
        self.connection_lock = threading.Lock()
        self.retry_at = None
        self.input_warned = False
        self.on_connection_change = lambda: None
        super().__init__(*args, **kwargs)

    def report(self, message):
        sys.stderr.write(f"\n[serial_term] {message}\n")
        sys.stderr.flush()

    def schedule_retry(self, error):
        super().close()
        self.retry_at = time.monotonic() + self.retry_delay
        self.on_connection_change()
        self.report(f"TCP connection lost or unavailable: {error}. "
                    f"Retrying in {self.retry_delay:g} seconds.")

    def open_once(self):
        # Bound reads and writes so keyboard handling and shutdown stay responsive.
        self.timeout = 0.1
        self.write_timeout = 1
        try:
            super().open()
        except (serial.SerialException, OSError) as error:
            self.schedule_retry(error)
        else:
            self.retry_at = None
            self.input_warned = False
            self.on_connection_change()
            self.report(f"TCP connected to {self.port}.")

    def open(self):
        self.cancelled.clear()
        with self.connection_lock:
            self.open_once()

    @property
    def in_waiting(self):
        # Let read() handle both EOF and reconnection, without a separate socket poll.
        return 0

    def read(self, size=1):
        if self.cancelled.is_set():
            return b""
        if self.retry_at is not None and time.monotonic() < self.retry_at:
            self.cancelled.wait(min(0.1, self.retry_at - time.monotonic()))
            return b""
        with self.connection_lock:
            if self.cancelled.is_set():
                return b""
            if self.retry_at is not None:
                self.open_once()
                return b""
            try:
                return super().read(size)
            except (serial.SerialException, OSError) as error:
                self.schedule_retry(error)
                return b""

    def ignore_input(self):
        if not self.input_warned:
            self.report("Input ignored while TCP is disconnected.")
            self.input_warned = True
        return 0

    def write(self, data):
        if not self.is_open:
            return self.ignore_input()
        with self.connection_lock:
            if not self.is_open:
                return self.ignore_input()
            try:
                return super().write(data)
            except (serial.SerialException, OSError) as error:
                self.schedule_retry(error)
                return self.ignore_input()

    def close(self):
        self.cancelled.set()
        with self.connection_lock:
            super().close()


class HighlightMiniterm(miniterm.Miniterm):
    def __init__(self, *args, highlight=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.highlighter = Highlighter()
        self.write_bytes = self.console.write_bytes
        if highlight:
            self.console.write_bytes = self.write_highlighted
        if isinstance(self.serial, ReconnectingSocket):
            self.serial.on_connection_change = self.reset_highlighter

    def reset_highlighter(self):
        self.write_bytes(self.highlighter.reset_color())
        self.highlighter = Highlighter()

    def write_highlighted(self, data):
        self.write_bytes(self.highlighter.feed(data))

    def stop(self):
        super().stop()
        if isinstance(self.serial, ReconnectingSocket):
            self.serial.cancelled.set()

    def close(self):
        try:
            self.write_bytes(self.highlighter.reset_color())
        finally:
            super().close()


def main():
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto",
                        help="host highlighting (default: auto; respects NO_COLOR)")
    parser.add_argument("--no-color", action="store_const", const="never", dest="color",
                        help="disable host highlighting; retain device ANSI sequences")
    parser.add_argument("--reconnect-delay", type=float, default=5, metavar="SECONDS",
                        help="TCP reconnect interval (default: 5 seconds)")
    parser.add_argument("--no-reconnect", action="store_true",
                        help="disable automatic socket:// reconnection")
    args, remaining = parser.parse_known_args()
    if not math.isfinite(args.reconnect_delay) or args.reconnect_delay <= 0:
        parser.error("--reconnect-delay must be a positive, finite number")
    if "--help" in remaining or "-h" in remaining:
        print(__doc__)
        print("Example: python tools/serial_term.py socket://localhost:23201")
        parser.print_help()
        print("\nAll standard miniterm options follow (defaults here: raw, CR, 115200).")

    enabled = args.color == "always" or (
        args.color == "auto" and sys.stdout.isatty()
        and "NO_COLOR" not in os.environ and os.environ.get("TERM") != "dumb"
    )
    original_miniterm, original_argv = miniterm.Miniterm, sys.argv
    original_serial_for_url = serial.serial_for_url

    def serial_for_url(url, *port_args, **port_kwargs):
        if args.no_reconnect or not url.lower().startswith("socket://"):
            return original_serial_for_url(url, *port_args, **port_kwargs)
        do_open = not port_kwargs.pop("do_not_open", False)
        port = ReconnectingSocket(None, *port_args, retry_delay=args.reconnect_delay,
                                  **port_kwargs)
        port.port = url
        port.from_url(url)  # Invalid URLs should fail immediately, not retry forever.
        if do_open:
            port.open()
        return port

    try:
        miniterm.Miniterm = partial(HighlightMiniterm, highlight=enabled)
        serial.serial_for_url = serial_for_url
        # Miniterm owns URL handling, keyboard shortcuts, port setup and I/O.
        sys.argv = [sys.argv[0], "--raw", "--eol", "CR", *remaining]
        miniterm.main(default_baudrate=115200)
    finally:
        miniterm.Miniterm, sys.argv = original_miniterm, original_argv
        serial.serial_for_url = original_serial_for_url


if __name__ == "__main__":
    main()
