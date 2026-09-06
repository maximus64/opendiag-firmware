#!/bin/sh
#
# Runs the suites and reports line coverage of the firmware sources.
#
# Invoked by the "coverage" CMake target, which passes COV_COMPILER, BUILD_DIR,
# SRC_DIR and FW_DIR in the environment. Configure with -DCOVERAGE=ON first:
#
#   cmake -S test/host -B test/host/build -DCOVERAGE=ON
#   cmake --build test/host/build --target coverage
#
# Only main/ is measured. The fakes and the framework are test scaffolding, and
# counting them would flatter the number.

set -e

: "${BUILD_DIR:=.}"
: "${FW_DIR:?FW_DIR is not set}"
: "${COV_COMPILER:=Clang}"

cd "$BUILD_DIR"

# Passed in by the CMake target so the list lives in one place.
: "${SUITE_LIST:=test_elm327_at test_elm327_can test_elm327_bytebus test_elm327_search test_comm_iface test_vif test_slcan test_j1850_pwm_codec}"
SUITES="$SUITE_LIST"

# Named one by one rather than as a directory: llvm-cov only filters on files.
SOURCES="$FW_DIR/elm327_at.c $FW_DIR/vif.c $FW_DIR/slcan.c $FW_DIR/comm_iface.c $FW_DIR/timer.c $FW_DIR/utility.c $FW_DIR/j1850_pwm_codec.c $FW_DIR/j1850_vpw_codec.c $FW_DIR/j1850_common.c"

for suite in $SUITES; do
    if [ ! -x "./$suite" ]; then
        echo "$suite is not built. Run cmake --build first." >&2
        exit 1
    fi
done

case "$COV_COMPILER" in
Clang|AppleClang)
    # macOS ships the LLVM tools inside the toolchain rather than on PATH.
    if command -v llvm-profdata >/dev/null 2>&1; then
        PROFDATA="llvm-profdata"
        COV="llvm-cov"
    elif command -v xcrun >/dev/null 2>&1; then
        PROFDATA="xcrun llvm-profdata"
        COV="xcrun llvm-cov"
    else
        echo "llvm-profdata not found. Install LLVM or the Xcode tools." >&2
        exit 1
    fi

    rm -f ./*.profraw ./coverage.profdata

    for suite in $SUITES; do
        LLVM_PROFILE_FILE="$BUILD_DIR/$suite.profraw" "./$suite" >/dev/null
    done

    $PROFDATA merge -sparse ./*.profraw -o coverage.profdata

    # Each binary carries its own copy of the included elm327_at.c, so every
    # one has to be named for the merged totals to be right. The first is
    # positional and the rest need -object.
    OBJECTS=""
    first=""
    for suite in $SUITES; do
        if [ -z "$first" ]; then
            first="./$suite"
        else
            OBJECTS="$OBJECTS -object ./$suite"
        fi
    done

    # shellcheck disable=SC2086
    $COV report $first $OBJECTS \
        -instr-profile=coverage.profdata \
        -show-region-summary=false \
        $SOURCES

    echo
    echo "Annotated source:"
    echo "  $COV show $first $OBJECTS -instr-profile=$BUILD_DIR/coverage.profdata $FW_DIR/elm327_at.c | less -R"
    ;;

GNU)
    if ! command -v gcov >/dev/null 2>&1; then
        echo "gcov not found." >&2
        exit 1
    fi

    for suite in $SUITES; do
        "./$suite" >/dev/null
    done

    if command -v gcovr >/dev/null 2>&1; then
        gcovr --root "$FW_DIR" --print-summary .
    else
        find . -name '*.gcda' -exec gcov -b -o {} \; >/dev/null
        echo "gcovr not installed; raw .gcov files are in $BUILD_DIR"
    fi
    ;;

*)
    echo "Coverage is not wired up for compiler '$COV_COMPILER'." >&2
    exit 1
    ;;
esac
