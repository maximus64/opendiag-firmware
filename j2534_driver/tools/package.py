#!/usr/bin/env python3
"""Create one deterministic Windows driver ZIP from checked build artifacts."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile

from check_wincompat import EXPORTS, inspect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    project = Path(__file__).resolve().parents[1]
    allowed = json.loads((project / "tools/xp-imports.json").read_text())["libraries"]
    report = json.loads((directory / "wincompat.json").read_text())
    files = {}
    for name in EXPORTS:
        if inspect(directory / name, allowed) != report["files"][name]:
            raise SystemExit(f"Compatibility report is stale: {name}")
        files[name] = (directory / name).read_bytes()
    # A release always ships the template, never a developer's local endpoint configuration.
    files["opendiag.ini"] = (project / "opendiag.ini").read_bytes()
    files["wincompat.json"] = (directory / "wincompat.json").read_bytes()
    for name in ("README.txt", "install.cmd", "uninstall.cmd", "opendiag-cdc-xp.inf"):
        text = (project / "packaging" / name).read_text()
        files[name] = text.replace("\n", "\r\n").encode("ascii")
    for name in ("j2534.h", "j2534_0404.h", "j2534_types.h"):
        files["include/" + name] = (project / "include" / name).read_bytes()
    files["COPYING"] = (project.parent / "LICENSE").read_bytes()
    files["nanopb-LICENSE.txt"] = (project.parent / "components/nanopb/upstream/LICENSE.txt").read_bytes()
    sums = "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n"
                   for name, data in sorted(files.items()))
    files["SHA256SUMS"] = sums.encode("ascii")
    epoch = max(315532800, int(os.environ.get("SOURCE_DATE_EPOCH", "315532800")))
    date = datetime.datetime.fromtimestamp(epoch, datetime.timezone.utc).timetuple()[:6]
    destination = directory / "opendiag-j2534-driver.zip"
    with tempfile.NamedTemporaryFile(dir=directory, suffix=".zip", delete=False) as temporary:
        temporary_path = Path(temporary.name)
    try:
        with zipfile.ZipFile(temporary_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for name, data in sorted(files.items()):
                entry = zipfile.ZipInfo(name, date)
                entry.create_system = 3
                entry.external_attr = 0o100644 << 16
                entry.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(entry, data, compress_type=zipfile.ZIP_DEFLATED, compresslevel=9)
        with zipfile.ZipFile(temporary_path) as archive:
            if archive.testzip() is not None:
                raise RuntimeError("ZIP integrity check failed")
        temporary_path.chmod(0o644)
        temporary_path.replace(destination)
    finally:
        temporary_path.unlink(missing_ok=True)
    print("Package:", destination)


if __name__ == "__main__":
    main()
