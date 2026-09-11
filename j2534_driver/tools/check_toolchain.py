#!/usr/bin/env python3
"""Reject Windows toolchains with the wrong architecture or C runtime."""
import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--cxx", required=True)
    args = parser.parse_args()
    for compiler in (args.cc, args.cxx):
        try:
            result = subprocess.run(
                [compiler, "-dM", "-E", "-include", "_mingw.h", "-"], input="",
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            raise SystemExit(f"Cannot inspect {compiler}: {error}") from error
        macros = result.stdout
        if "#define __i386__ " not in macros or "#define __MINGW32__ " not in macros:
            raise SystemExit(f"{compiler}: use an i686 MinGW compiler for the 32-bit Windows ABI")
        if "#define _UCRT" in macros:
            raise SystemExit(f"{compiler}: UCRT is not the XP release runtime; use the Compose builder or an MSVCRT toolchain")
    print("PASS toolchain: i686 MinGW / MSVCRT")


if __name__ == "__main__":
    main()
