#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Package ESP-IDF flash images for the OpenDIAG web installer."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import tempfile
import zipfile


def embedded_version(image):
    # esp_app_desc_t follows the 24-byte image and 8-byte segment headers.
    if len(image) < 32 + 256 or image[0] != 0xE9:
        raise ValueError("Invalid ESP application image")
    segment_size = struct.unpack_from("<I", image, 28)[0]
    if segment_size < 256 or segment_size > len(image) - 32:
        raise ValueError("Invalid application descriptor segment")
    if struct.unpack_from("<I", image, 32)[0] != 0xABCD5432:
        raise ValueError("Missing ESP application descriptor")
    version = image[48:80]
    if b"\0" not in version:
        raise ValueError("Unterminated application version")
    version = version.split(b"\0", 1)[0].decode("utf-8")
    if not version:
        raise ValueError("Empty application version")
    return version


def package_firmware(build_dir):
    build_dir = Path(build_dir).resolve()
    project = json.loads((build_dir / "project_description.json").read_text())
    flash = json.loads((build_dir / "flasher_args.json").read_text())
    if project["target"] != "esp32s3":
        raise ValueError("OpenDIAG packages require an ESP32-S3 build")

    flash_files = flash["flash_files"]
    for role in ("bootloader", "partition-table", "app"):
        image = flash[role]
        if flash_files.get(image["offset"]) != image["file"]:
            raise ValueError(f"Missing {role} in flash_files")

    files = {}
    parts = []
    end = 0
    for address, filename in sorted(flash_files.items(), key=lambda item: int(item[0], 0)):
        source = build_dir / filename
        name = source.name
        offset = int(address, 0)
        if name in files or name in ("manifest.json", "SHA256SUMS"):
            raise ValueError(f"Duplicate package filename: {name}")
        data = source.read_bytes()
        if not data or offset < end:
            raise ValueError(f"Empty or overlapping flash image: {filename}")
        end = offset + len(data)
        files[name] = data
        parts.append({"path": name, "offset": offset})

    app = files[Path(flash["app"]["file"]).name]
    version = embedded_version(app)

    manifest = {
        "name": "OpenDiag Firmware",
        "version": version,
        "new_install_prevent_erase": True,
        "builds": [{"chipFamily": "ESP32-S3", "parts": parts}],
    }
    files["manifest.json"] = (json.dumps(manifest, indent=2) + "\n").encode()
    files["SHA256SUMS"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n"
        for name, data in sorted(files.items())
    ).encode()

    output = build_dir / "firmware"
    archive = build_dir / "opendiag-firmware.zip"
    with tempfile.TemporaryDirectory(prefix="firmware-package-", dir=build_dir) as temporary:
        stage = Path(temporary) / "firmware"
        stage.mkdir()
        staged_archive = Path(temporary) / archive.name
        with zipfile.ZipFile(staged_archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
            for name, data in sorted(files.items()):
                (stage / name).write_bytes(data)
                info = zipfile.ZipInfo(name)
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                bundle.writestr(info, data)
        if output.exists():
            shutil.rmtree(output)
        stage.replace(output)
        staged_archive.replace(archive)
    return output, archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path("build"))
    args = parser.parse_args()
    try:
        output, archive = package_firmware(args.build_dir)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Firmware packaging failed: {error}\n")
    print(f"Firmware directory: {output}\nFirmware archive: {archive}")


if __name__ == "__main__":
    main()
