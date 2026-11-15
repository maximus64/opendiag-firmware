#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Write boards.json, the board list the firmware update page offers.

Usage: site_board_list.py <site>/update/firmware

The directory holds one unpacked firmware package per board, in a subdirectory
named after the board. Both the Docs workflow and tools/build_site.py call this,
so the published site and a local build validate boards/ the same way.
"""

import json
from pathlib import Path
import sys


BOARDS_DIR = Path(__file__).resolve().parents[1] / "boards"


def download_steps(board, info):
    # The two steps that put the board in firmware-download mode. They differ between boards, and
    # the update page lays out exactly two.
    steps = info.get("download_steps")
    if (
        not isinstance(steps, list)
        or len(steps) != 2
        or not all(
            isinstance(step, dict)
            and isinstance(step.get("title"), str)
            and isinstance(step.get("text"), str)
            for step in steps
        )
    ):
        raise ValueError(
            f"{board}/board.json needs download_steps: two objects with title and text"
        )
    return steps


def finish_note(board, info):
    note = info.get("finish_note")
    if note is not None and not isinstance(note, str):
        raise ValueError(f"{board}/board.json finish_note must be a string")
    return note


def info_url(board, info):
    url = info.get("info_url")
    if url is not None and not (isinstance(url, str) and url.startswith("https://")):
        raise ValueError(f"{board}/board.json info_url must be an https:// URL")
    return url


def write_board_list(firmware_dir, boards_dir=BOARDS_DIR):
    firmware_dir = Path(firmware_dir)
    published = {path.name for path in firmware_dir.iterdir() if path.is_dir()}
    # board.json is UTF-8; the platform default encoding is not on every host.
    boards = {
        path.parent.name: json.loads(path.read_text(encoding="utf-8"))
        for path in Path(boards_dir).glob("*/board.json")
    }

    # Firmware for a board that is not in boards/ is an error. A board with no firmware is left off
    # the list, but the default board must have some.
    unknown = sorted(published - set(boards))
    if unknown:
        raise ValueError(f"Firmware packages without a board in boards/: {unknown}")
    listed = {board: info for board, info in boards.items() if board in published}
    if not listed:
        raise ValueError("There is no firmware package for any board in boards/")
    defaults = sorted(board for board, info in boards.items() if info.get("default") is True)
    if len(defaults) != 1:
        raise ValueError(f'Exactly one board.json must have "default": true, found {defaults}')
    if defaults[0] not in listed:
        raise ValueError(f"The default board {defaults[0]} has no firmware package")
    for board, info in listed.items():
        if not isinstance(info.get("name"), str):
            raise ValueError(f"{board}/board.json needs a name string")

    # The update page starts on the first entry: the default board, then the rest in board id order.
    entries = [
        {
            "name": info["name"],
            "default": board == defaults[0],
            "manifest": f"firmware/{board}/manifest.json",
            "download_steps": download_steps(board, info),
            "finish_note": finish_note(board, info),
            "info_url": info_url(board, info),
        }
        for board, info in sorted(
            listed.items(), key=lambda item: (item[0] != defaults[0], item[0])
        )
    ]
    (firmware_dir / "boards.json").write_text(
        json.dumps(entries, indent=2) + "\n", encoding="utf-8"
    )


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {Path(sys.argv[0]).name} <site>/update/firmware")
    try:
        write_board_list(sys.argv[1])
    except (OSError, ValueError) as error:
        raise SystemExit(f"Board list not written: {error}") from None


if __name__ == "__main__":
    main()
