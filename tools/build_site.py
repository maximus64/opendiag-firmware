#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Build every board's firmware in Docker and assemble the website around it.

The result, under build/github-site/, is the documentation site with the
firmware update page serving the firmware just built instead of the latest
release. The Python running this script needs Zensical; Docker with Compose
(the plugin or the standalone docker-compose command) builds the firmware.
"""

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

from site_board_list import write_board_list


ROOT = Path(__file__).resolve().parents[1]
# Zensical has no output option; this is site_dir in mkdocs.yml.
SITE_DIR = ROOT / "build/github-site"
PACKAGE_DIR = ROOT / "build/docker"
PACKAGE_PREFIX = "opendiag-firmware-"
UPDATE_FIRMWARE_DIR = SITE_DIR / "update/firmware"


def run(command):
    print(f"\n==> {' '.join(str(part) for part in command)}", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def responds(command):
    try:
        result = subprocess.run(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
        )
    except OSError:
        return False
    return result.returncode == 0


def find_compose():
    for command in (["docker", "compose"], ["docker-compose"]):
        if responds([*command, "version"]):
            return command
    raise SystemExit("Docker with Compose is required to build the firmware.")


def check_zensical():
    if not responds([sys.executable, "-m", "zensical", "--version"]):
        raise SystemExit(
            f"Zensical is not available in {sys.executable}. Install it there with:\n"
            f"  {sys.executable} -m pip install zensical"
        )


def install_package(package, destination):
    destination.mkdir(parents=True)
    with zipfile.ZipFile(package) as archive:
        archive.extractall(destination)
    for line in (destination / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        expected, name = line.split(maxsplit=1)
        name = name.lstrip("*")
        if hashlib.sha256((destination / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"SHA-256 mismatch for {destination.name}/{name}")


def install_firmware():
    packages = {
        path.stem.removeprefix(PACKAGE_PREFIX): path
        for path in PACKAGE_DIR.glob(f"{PACKAGE_PREFIX}*.zip")
    }
    # build/docker/ outlives a build, so a ZIP for a board that has since been removed or renamed
    # can still be there. The firmware build makes one ZIP for each boards/<id>/ with a pinout.h.
    built_boards = {path.parent.name for path in ROOT.glob("boards/*/pinout.h")}
    if set(packages) != built_boards:
        raise ValueError(
            "Firmware packages in build/docker do not match boards/: "
            f"missing={sorted(built_boards - set(packages))}, "
            f"unexpected={sorted(set(packages) - built_boards)}"
        )
    for board, package in sorted(packages.items()):
        install_package(package, UPDATE_FIRMWARE_DIR / board)
    write_board_list(UPDATE_FIRMWARE_DIR)


def main():
    argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    ).parse_args()
    compose = find_compose()
    check_zensical()
    # The firmware service binds build/docker and does not create it.
    PACKAGE_DIR.mkdir(parents=True, exist_ok=True)
    try:
        # The site takes seconds and the firmware minutes, so a docs error surfaces first. Zensical
        # empties the site directory, which also clears firmware left by an earlier run.
        run([sys.executable, "-m", "zensical", "build", "--strict"])
        run([*compose, "run", "--rm", "--build", "firmware"])
        install_firmware()
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode or 1) from None
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        raise SystemExit(f"Website assembly failed: {error}") from None

    site = SITE_DIR.relative_to(ROOT)
    print(
        f"\nSite built in {site}\n"
        f"Preview: python3 -m http.server 8000 -d {site}\n"
        "Updater: http://localhost:8000/update/"
    )


if __name__ == "__main__":
    main()
