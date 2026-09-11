#!/usr/bin/env python3
"""Exercise the binary gate on synthetic PE files, using only the native host."""
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from check_wincompat import inspect


def image():
    data = bytearray(1536)
    data[:2] = b"MZ"
    struct.pack_into("<I", data, 0x3c, 0x80)
    data[0x80:0x84] = b"PE\0\0"
    struct.pack_into("<HHIIIHH", data, 0x84, 0x14c, 1, 0, 0, 0, 224, 0x102)
    struct.pack_into("<H", data, 0x98, 0x10b)
    struct.pack_into("<HH", data, 0x98 + 40, 5, 1)
    struct.pack_into("<HH", data, 0x98 + 48, 5, 1)
    struct.pack_into("<I", data, 0x98 + 60, 512)
    struct.pack_into("<H", data, 0x98 + 68, 2)
    struct.pack_into("<I", data, 0x98 + 92, 16)
    struct.pack_into("<II", data, 0x98 + 104, 0x1000, 40)
    struct.pack_into("<IIII", data, 0x178 + 8, 1024, 0x1000, 1024, 512)
    struct.pack_into("<IIIII", data, 512, 0x1040, 0, 0, 0x1060, 0x1050)
    struct.pack_into("<I", data, 512 + 64, 0x1080)
    data[512 + 96:512 + 109] = b"KERNEL32.dll\0"
    data[512 + 130:512 + 136] = b"Sleep\0"
    return data


class CompatibilityTests(unittest.TestCase):
    def inspect(self, data):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "opendiag_config.exe"
            path.write_bytes(data)
            return inspect(path, {"kernel32.dll": ["Sleep"]})

    def test_allowed_image(self):
        self.assertEqual(self.inspect(image())["imports"], {"kernel32.dll": ["Sleep"]})

    def test_reject_wrong_architecture(self):
        data = image()
        struct.pack_into("<H", data, 0x84, 0x8664)
        with self.assertRaisesRegex(ValueError, "x86"):
            self.inspect(data)

    def test_reject_newer_loader(self):
        data = image()
        struct.pack_into("<HH", data, 0x98 + 48, 6, 0)
        with self.assertRaisesRegex(ValueError, "loader"):
            self.inspect(data)

    def test_reject_newer_function(self):
        data = image()
        data[512 + 130:512 + 145] = b"GetTickCount64\0"
        with self.assertRaisesRegex(ValueError, "Imports need XP review"):
            self.inspect(data)

    def test_reject_unapproved_runtime(self):
        data = image()
        data[512 + 96:512 + 109] = b"ucrtbase.dll\0"
        with self.assertRaisesRegex(ValueError, "runtime DLL"):
            self.inspect(data)

    def test_reject_ordinal_import(self):
        data = image()
        struct.pack_into("<I", data, 512 + 64, 0x80000001)
        with self.assertRaisesRegex(ValueError, "Ordinal import"):
            self.inspect(data)

    def test_reject_delay_import(self):
        data = image()
        struct.pack_into("<II", data, 0x98 + 96 + 13 * 8, 0x1100, 32)
        with self.assertRaisesRegex(ValueError, "Delay imports"):
            self.inspect(data)

    def test_reject_unmapped_import(self):
        data = image()
        struct.pack_into("<I", data, 512 + 64, 0x800000)
        with self.assertRaisesRegex(ValueError, "Unmapped"):
            self.inspect(data)


if __name__ == "__main__":
    unittest.main()
