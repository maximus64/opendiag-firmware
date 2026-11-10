#!/usr/bin/env python3
"""Real firmware RPC dispatch with host bus fakes; no diagnostic behavior model."""
import ctypes as c
import threading

import mock_peer

# j2534_core_request() result for a request answered later (J2534_CORE_HELD).
HELD = c.c_size_t(-1).value


class Firmware:
    def __init__(self, path):
        self.core = c.CDLL(str(path))
        self.lock = threading.RLock()
        self.auto_poll = True
        signatures = {
            "contract_reset": (c.c_int, []),
            "contract_stop": (c.c_int, []),
            "contract_inject": (c.c_int, [c.c_uint, c.c_void_p, c.c_size_t]),
            "contract_sent_count": (c.c_int, [c.c_uint]),
            "contract_sent": (c.c_int, [c.c_uint, c.c_uint, c.c_void_p, c.c_size_t]),
            "j2534_core_request": (c.c_size_t, [c.c_uint8, c.c_void_p, c.c_size_t, c.c_void_p]),
            "j2534_core_take_held": (c.c_bool, [c.c_void_p, c.POINTER(c.c_size_t)]),
            "j2534_core_next_wake": (c.c_bool, [c.POINTER(c.c_uint32)]),
            "esp_timer_get_time": (c.c_int64, []),
            "j2534_core_poll": (None, []),
            "fake_clock_advance_ms": (None, [c.c_uint32]),
            "fake_bus_send_duration": (None, [c.c_int, c.c_uint32]),
        }
        for name, (result, arguments) in signatures.items():
            function = getattr(self.core, name)
            function.restype = result
            function.argtypes = arguments
        if self.core.contract_reset():
            raise RuntimeError("Firmware fixture initialization failed")

    def stop(self):
        with self.lock:
            if self.core.contract_stop():
                raise RuntimeError("Firmware fixture did not release its resources")

    def respond(self, request):
        field = request.DESCRIPTOR.fields_by_name[request.WhichOneof("command")]
        data = request.SerializeToString()
        output = c.create_string_buffer(4608)
        with self.lock:
            if self.auto_poll:
                self.core.j2534_core_poll()
            size = self.core.j2534_core_request(field.number - 1, data, len(data), output)
            if size == HELD:
                size = self._answer_held(output)
            elif self.auto_poll:
                self.core.j2534_core_poll()
        return mock_peer.pb.Response.FromString(output.raw[:size])

    def _answer_held(self, output):
        """Run the link task's loop for a request the core is holding.

        The caller is blocked in the driver, so nothing can arrive: time moves
        to each deadline the core reports until the request is answered."""
        size = c.c_size_t()
        due = c.c_uint32()
        while True:
            self.core.j2534_core_poll()
            if self.core.j2534_core_take_held(output, c.byref(size)):
                return size.value
            if not self.core.j2534_core_next_wake(c.byref(due)):
                raise RuntimeError("held request with no deadline")
            now = self.core.esp_timer_get_time() & 0xffffffff
            left_us = (due.value - now) & 0xffffffff
            self.core.fake_clock_advance_ms(max(1, -(-left_us // 1000)))

    def poll(self):
        with self.lock:
            self.core.j2534_core_poll()

    def advance(self, milliseconds):
        with self.lock:
            self.core.fake_clock_advance_ms(milliseconds)
            self.core.j2534_core_poll()

    def inject(self, data, protocol=3):
        # Packet fakes do not validate checksums; supply a trailer for the core
        # to strip. Its value is the ISO9141 checksum, not a J1850 CRC model.
        if protocol != 5:
            data += bytes([sum(data) & 0xff])
        with self.lock:
            if self.core.contract_inject(protocol, data, len(data)):
                raise ValueError("Invalid fake bus frame")
            self.core.j2534_core_poll()

    def sent(self, protocol=3):
        with self.lock:
            count = self.core.contract_sent_count(protocol)
            if count < 0:
                raise ValueError("Unsupported fake bus")
            frames = []
            for index in range(count):
                data = c.create_string_buffer(32)
                size = self.core.contract_sent(protocol, index, data, len(data))
                if size < 0:
                    raise RuntimeError("Cannot read fake bus transmit log")
                frames.append(data.raw[:size])
            return frames

    def send_duration(self, milliseconds):
        with self.lock:
            self.core.fake_bus_send_duration(0, milliseconds)

    def peer_type(self):
        firmware = self

        class Peer(mock_peer.Peer):
            def respond(self, request):
                return firmware.respond(request)

        return Peer
