#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
#
# Stages the firmware update page the way .github/workflows/docs.yml publishes it, then serves it on
# 127.0.0.1 so the page can be tested before a release. The board packages come from the Docker
# release build, which writes build/docker/opendiag-firmware-<board>.zip for each board:
#   mkdir -p build/docker && docker compose run --rm --build firmware
set -euo pipefail

usage() {
    echo "Usage: tools/updater/serve-update.sh [--port <n>] [--out <dir>]"
    echo "Stages docs/update and the board ZIPs in build/docker into <dir>/update, then serves <dir>."
    echo "Defaults: --port 8000, --out build/updater. Open http://127.0.0.1:<port>/update/ and press Ctrl-C to stop."
}

port=8000
out=build/updater
while (( $# > 0 )); do
    case "$1" in
        --port)
            if (( $# < 2 )) || [[ ! $2 =~ ^[0-9]+$ ]]; then
                usage >&2
                exit 2
            fi
            port=$2
            shift
            ;;
        --out)
            if (( $# < 2 )) || [[ -z $2 ]]; then
                usage >&2
                exit 2
            fi
            out=$2
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
    shift
done

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
case $out in
    /*) ;;
    *) out="$repo/$out" ;;
esac

if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is required to stage and serve the update page." >&2
    exit 1
fi

# Only boards with a ZIP are staged. A board without one is left off the list, as the docs build does.
staged=()
for board_json in boards/*/board.json; do
    board=$(basename "$(dirname "$board_json")")
    if [[ -f "build/docker/opendiag-firmware-$board.zip" ]]; then
        staged+=("$board")
    else
        echo "Skipping $board: build/docker/opendiag-firmware-$board.zip is missing." >&2
    fi
done
if (( ${#staged[@]} == 0 )); then
    echo "No board ZIPs in build/docker. Run the Docker release build first." >&2
    exit 1
fi

# Only update/ is replaced, so the output folder itself can stay open in another program.
rm -rf -- "$out/update"
mkdir -p "$out/update/firmware"
cp docs/update/index.html docs/update/logo.svg "$out/update/"

for board in "${staged[@]}"; do
    package="build/docker/opendiag-firmware-$board.zip"
    dest="$out/update/firmware/$board"
    mkdir -p "$dest"
    python3 -I -m zipfile -e "$package" "$dest"
    (cd "$dest" && sha256sum --check SHA256SUMS)
done

# Same board-list logic as the "Write the update page board list" step in docs.yml.
python3 -I - "$out/update/firmware" <<'PY'
import json
import sys
from pathlib import Path
firmware = Path(sys.argv[1])
published = {path.name for path in firmware.iterdir() if path.is_dir()}
boards = {
    path.parent.name: json.loads(path.read_text())
    for path in Path("boards").glob("*/board.json")
}
unknown = sorted(published - set(boards))
if unknown:
    raise SystemExit(f"Release ZIPs without a board in boards/: {unknown}")
listed = {board: info for board, info in boards.items() if board in published}
if not listed:
    raise SystemExit("The latest release has no ZIP for any board in boards/")
defaults = sorted(board for board, info in boards.items() if info["default"] is True)
if len(defaults) != 1:
    raise SystemExit(f"Exactly one board.json must have \"default\": true, found {defaults}")
if defaults[0] not in listed:
    raise SystemExit(f"The default board {defaults[0]} has no ZIP in the latest release")
def download_steps(board, info):
    steps = info.get("download_steps")
    if not isinstance(steps, list) or len(steps) != 2 or not all(
        isinstance(step, dict) and isinstance(step.get("title"), str) and isinstance(step.get("text"), str)
        for step in steps
    ):
        raise SystemExit(f"{board}/board.json needs download_steps: two objects with title and text")
    return steps
def finish_note(board, info):
    note = info.get("finish_note")
    if note is not None and not isinstance(note, str):
        raise SystemExit(f"{board}/board.json finish_note must be a string")
    return note
entries = [
    {
        "name": info["name"],
        "default": info["default"],
        "manifest": f"firmware/{board}/manifest.json",
        "download_steps": download_steps(board, info),
        "finish_note": finish_note(board, info),
    }
    for board, info in sorted(listed.items(), key=lambda item: (not item[1]["default"], item[0]))
]
(firmware / "boards.json").write_text(json.dumps(entries, indent=2) + "\n")
PY

echo "Staged ${staged[*]} in $out/update"
echo "Serving http://127.0.0.1:$port/update/ (Ctrl-C to stop)"
exec python3 -m http.server "$port" --bind 127.0.0.1 --directory "$out"
