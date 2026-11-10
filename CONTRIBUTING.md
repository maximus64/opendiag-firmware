# Contributing to OpenDIAG

Thanks for helping. The full guide is [docs/developers.md](docs/developers.md), also published at
[maximus64.github.io/opendiag-firmware/developers](https://maximus64.github.io/opendiag-firmware/developers/).
It covers the architecture, building, the host tests, adding an AT or shell command, and the
list of open work.

In short:

- **Small pull requests.** One feature or fix each, branched from `main`. Stack dependent ones
  and say so.
- **Commit subjects:** `area: what it does` (`elm327: add AT CRA reply filter`). Put docs in
  their own commit.
- **Tests:** a host test that fails without your change (`ctest --test-dir test/host/build`).
- **Docs:** update `docs/manual.md` and `docs/at_commands.md` for anything a user sees, and check
  with `mkdocs build --strict`.
- **Format:** `tools/format.sh` on the code you touched.
- **Vehicle safety:** never add a path that drives a CAN bus without listening first, energises a
  pin from BLE or the control plane, or transmits by default. See
  [Rules that keep a vehicle safe](docs/developers.md#rules-that-keep-a-vehicle-safe).

By contributing you agree that your work is released under the
[GNU GPL, version 3](LICENSE).
