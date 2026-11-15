#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

usage() {
    echo "Usage: docker compose run --rm --build firmware [--verify] [--board <id>]"
    echo "Create the output directory first: mkdir -p build/docker"
    echo "Builds every board under boards/ and exports opendiag-firmware-<board>.zip for each."
    echo "--board <id> builds and exports only that board."
    echo "--verify builds each selected board twice from scratch and compares the ZIPs."
}

verify=false
selected_board=""
while (( $# > 0 )); do
    case "$1" in
        --verify) verify=true ;;
        --board)
            if (( $# < 2 )) || [[ -z $2 ]]; then
                usage >&2
                exit 2
            fi
            selected_board=$2
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
    shift
done

if [[ $(uname -m) != x86_64 ]]; then
    echo "The release builder requires platform: linux/amd64." >&2
    exit 1
fi

if [[ ! -d .git ]] || [[ $(git rev-parse --is-shallow-repository) != false ]]; then
    echo "Use a full Git clone with tags and recursive submodules." >&2
    exit 1
fi
submodules=$(git submodule status --recursive)
if grep -q '^[-+U]' <<< "$submodules"; then
    echo "Initialize the pinned submodules: git submodule update --init --recursive" >&2
    exit 1
fi

output_dir=${OPENDIAG_OUTPUT_DIR:-/output}
if [[ ! -d "$output_dir" || ! -w "$output_dir" ]]; then
    echo "Create the output directory first: mkdir -p build/docker" >&2
    exit 1
fi
if [[ -e build ]]; then
    echo "A clean container is required: docker compose run --rm --build firmware" >&2
    exit 1
fi

export SOURCE_DATE_EPOCH
SOURCE_DATE_EPOCH=$(git log -1 --format=%ct)

# A board is a boards/<name>/ directory holding pinout.h, the file main/CMakeLists.txt requires.
boards=()
for board_dir in boards/*/; do
    if [[ -f "$board_dir/pinout.h" ]]; then
        boards+=("$(basename "$board_dir")")
    fi
done
if (( ${#boards[@]} == 0 )); then
    echo "No boards with pinout.h found under boards/." >&2
    exit 1
fi
mapfile -t boards < <(printf '%s\n' "${boards[@]}" | sort)

# Exactly one board may be the default, so anything that picks a default board is unambiguous.
default_count=$(python -I -c '
import json
import sys
count = 0
for path in sys.argv[1:]:
    with open(path, encoding="utf-8") as f:
        if json.load(f).get("default") is True:
            count += 1
print(count)
' boards/*/board.json)
if [[ "$default_count" != 1 ]]; then
    echo "Exactly one boards/<id>/board.json must set \"default\": true." >&2
    exit 1
fi

if [[ -n "$selected_board" ]]; then
    if ! grep -qxF -- "$selected_board" <<< "$(printf '%s\n' "${boards[@]}")"; then
        echo "Unknown board: $selected_board. Boards with pinout.h under boards/: ${boards[*]}" >&2
        exit 1
    fi
    boards=("$selected_board")
fi
echo "Boards: ${boards[*]}"

cmake -S test/host -B build/host -G Ninja
cmake --build build/host --parallel 2
ctest --test-dir build/host --output-on-failure --timeout 30

cp dependencies.lock build/dependencies.lock
build_package() {
    local board=$1
    local build_dir=$2

    # Keep developer menuconfig settings out of release builds.
    idf.py -B "$build_dir" -D "SDKCONFIG=$PWD/$build_dir/sdkconfig" \
        -D "SDKCONFIG_DEFAULTS=$PWD/sdkconfig.defaults" -D "OPENDIAG_BOARD=$board" reconfigure
    if ! cmp -s dependencies.lock build/dependencies.lock; then
        echo "ESP-IDF changed dependencies.lock; update and commit it before building." >&2
        exit 1
    fi
    grep -qx 'CONFIG_APP_REPRODUCIBLE_BUILD=y' "$build_dir/sdkconfig"
    idf.py -B "$build_dir" firmware-package
    cmp dependencies.lock build/dependencies.lock

    (cd "$build_dir/firmware" && sha256sum --check SHA256SUMS)
    python -m zipfile --test "$build_dir/opendiag-firmware.zip"
}

for board in "${boards[@]}"; do
    build_package "$board" "build/$board/first"
    if "$verify"; then
        build_package "$board" "build/$board/second"
        cmp "build/$board/first/opendiag-firmware.zip" "build/$board/second/opendiag-firmware.zip"
        echo "Reproducibility verified for $board: both clean builds produced identical ZIPs."
    fi
done

# Export only after every board has built, so a failure leaves no partial set of ZIPs.
for board in "${boards[@]}"; do
    staged_archive=$(mktemp "$output_dir/.opendiag-firmware.XXXXXX")
    trap 'rm -f -- "$staged_archive"' EXIT
    cp "build/$board/first/opendiag-firmware.zip" "$staged_archive"
    chmod 644 "$staged_archive"
    chown --reference="$output_dir" "$staged_archive"
    mv -f -- "$staged_archive" "$output_dir/opendiag-firmware-$board.zip"
    echo "Firmware package: $output_dir/opendiag-firmware-$board.zip"
done
