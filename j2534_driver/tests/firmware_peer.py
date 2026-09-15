#!/usr/bin/env python3
"""Real firmware RPC dispatch with host bus fakes; no diagnostic behavior model."""
import ctypes as c
import threading

import mock_peer


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
            if self.auto_poll:
                self.core.j2534_core_poll()
        return mock_peer.pb.Response.FromString(output.raw[:size])

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
