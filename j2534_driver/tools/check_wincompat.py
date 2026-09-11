#!/usr/bin/env python3
"""Check PE architecture, XP loader requirements, named imports and J2534 exports."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


class PE:
    def __init__(self, data):
        self.data = data
        if data[:2] != b"MZ":
            raise ValueError("Missing DOS header")
        pe = self.unpack("I", 0x3c)[0]
        if data[pe:pe + 4] != b"PE\0\0":
            raise ValueError("Missing PE signature")
        machine, count, _, _, _, optional_size, flags = self.unpack("HHIIIHH", pe + 4)
        if machine != 0x14c:
            raise ValueError("Expected an x86 PE image")
        self.dll = bool(flags & 0x2000)
        optional = pe + 24
        if optional_size < 224 or self.unpack("H", optional)[0] != 0x10b:
            raise ValueError("Expected a PE32 optional header")
        self.os_version = self.unpack("HH", optional + 40)
        self.subsystem_version = self.unpack("HH", optional + 48)
        self.subsystem = self.unpack("H", optional + 68)[0]
        self.headers_size = self.unpack("I", optional + 60)[0]
        self.directories = [self.unpack("II", optional + 96 + index * 8) for index in range(16)]
        self.sections = []
        for index in range(count):
            offset = optional + optional_size + index * 40
            virtual_size, rva, raw_size, raw_offset = self.unpack("IIII", offset + 8)
            self.sections.append((rva, raw_size, raw_offset))

    def unpack(self, pattern, offset):
        if offset < 0:
            raise ValueError("Negative file offset")
        return struct.unpack_from("<" + pattern, self.data, offset)

    def offset(self, rva, size=1):
        if rva < self.headers_size and rva + size <= len(self.data):
            return rva
        for base, length, offset in self.sections:
            if base <= rva and rva + size <= base + length:
                result = offset + rva - base
                if result + size <= len(self.data):
                    return result
        raise ValueError(f"Unmapped RVA 0x{rva:x}")

    def string(self, rva):
        offset = self.offset(rva)
        end = self.data.find(b"\0", offset, offset + 1024)
        if end < 0:
            raise ValueError("Unterminated PE string")
        return self.data[offset:end].decode("ascii")

    def imports(self):
        if self.directories[13] != (0, 0):
            raise ValueError("Delay imports require a separate XP compatibility review")
        rva, size = self.directories[1]
        if not rva or size < 20:
            raise ValueError("Missing import directory")
        imports = {}
        for index in range(size // 20):
            entry = self.unpack("IIIII", self.offset(rva + index * 20, 20))
            if entry == (0, 0, 0, 0, 0):
                return imports
            thunk, _, _, name, first_thunk = entry
            library = self.string(name).lower()
            symbols = imports.setdefault(library, set())
            for item in range(4096):
                value = self.unpack("I", self.offset((thunk or first_thunk) + item * 4, 4))[0]
                if not value:
                    break
                if value & 0x80000000:
                    raise ValueError(f"Ordinal import in {library} needs explicit review")
                symbols.add(self.string(value + 2))
            else:
                raise ValueError("Unterminated import thunk table")
        raise ValueError("Unterminated import directory")

    def exports(self):
        rva, size = self.directories[0]
        if not rva:
            return {}, set()
        directory = self.offset(rva, 40)
        base, count, name_count, functions, names, ordinals = self.unpack("IIIIII", directory + 16)
        if count > 4096 or name_count > count:
            raise ValueError("Invalid export counts")
        present = set()
        for index in range(count):
            address = self.unpack("I", self.offset(functions + index * 4, 4))[0]
            if address:
                if rva <= address < rva + size:
                    raise ValueError("Forwarded API exports are not supported")
                self.offset(address)
                present.add(base + index)
        exports = {}
        for index in range(name_count):
            name = self.string(self.unpack("I", self.offset(names + index * 4, 4))[0])
            ordinal = self.unpack("H", self.offset(ordinals + index * 2, 2))[0] + base
            if name in exports or ordinal not in present:
                raise ValueError("Invalid or duplicate named export")
            exports[name] = ordinal
        return exports, present


COMMON_EXPORTS = {
    "PassThruOpen": 1, "PassThruClose": 2, "PassThruConnect": 3,
    "PassThruDisconnect": 4, "PassThruReadMsgs": 5, "PassThruStartPeriodicMsg": 7,
    "PassThruStopPeriodicMsg": 8, "PassThruStartMsgFilter": 9, "PassThruStopMsgFilter": 10,
    "PassThruSetProgrammingVoltage": 11, "PassThruReadVersion": 12,
    "PassThruGetLastError": 13, "PassThruIoctl": 14,
}
EXPORTS = {
    "opendiag32.dll": dict(COMMON_EXPORTS, PassThruQueueMsgs=15, PassThruScanForDevices=16,
                           PassThruGetNextDevice=17, PassThruLogicalConnect=18,
                           PassThruLogicalDisconnect=19, PassThruSelect=20),
    "odg40432.dll": dict(COMMON_EXPORTS, PassThruWriteMsgs=6),
    "opendiag_config.exe": {},
}


def inspect(path, allowed):
    data = path.read_bytes()
    pe = PE(data)
    if pe.dll != (path.suffix == ".dll"):
        raise ValueError("DLL/executable characteristic mismatch")
    if pe.os_version > (5, 1) or pe.subsystem_version != (5, 1):
        raise ValueError(f"XP loader version mismatch: OS {pe.os_version}, subsystem {pe.subsystem_version}")
    if pe.subsystem != 2:
        raise ValueError("Expected the Windows GUI subsystem")
    imports = pe.imports()
    for library, symbols in imports.items():
        if library not in allowed:
            raise ValueError(f"Unapproved runtime DLL: {library}")
        unknown = symbols - set(allowed[library])
        if unknown:
            raise ValueError(f"Imports need XP review: {library}: {', '.join(sorted(unknown))}")
    exports, ordinals = pe.exports()
    expected = EXPORTS[path.name]
    if exports != expected or ordinals != set(expected.values()):
        raise ValueError(f"J2534 export names/ordinals differ: {exports}")
    return {"sha256": hashlib.sha256(data).hexdigest(), "architecture": "x86",
            "subsystem": "5.01", "exports": exports,
            "imports": {name: sorted(values) for name, values in sorted(imports.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    policy = json.loads(Path(__file__).with_name("xp-imports.json").read_text())
    report = {"baseline": "Windows XP SP3 x86", "runtime_validated": False, "files": {}}
    try:
        for name in EXPORTS:
            report["files"][name] = inspect(args.directory / name, policy["libraries"])
            print(f"PASS {name}: x86, XP loader versions, imports and exports")
    except (ValueError, OSError, struct.error) as error:
        raise SystemExit(f"Windows compatibility check failed: {error}") from error
    (args.directory / "wincompat.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
