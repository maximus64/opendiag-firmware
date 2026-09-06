#!/bin/sh
#
# Formats the firmware sources with clang-format (see .clang-format).
#
# Invoked by the "format" / "format-check" targets in CMakeLists.txt (main
# build) and test/host/CMakeLists.txt (host tests), or directly:
#
#   tools/format.sh            # reformat in place
#   tools/format.sh --check    # fail without touching anything
#
# Covers main/ and test/host/, skipping test/host/build/ generated files.

set -e

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR/.."

MODE="-i"
if [ "$1" = "--check" ]; then
    MODE="--dry-run --Werror"
fi

find main test/host \( -iname '*.c' -o -iname '*.h' \) -not -path '*/build/*' -print0 \
    | xargs -0 clang-format $MODE
