# Two stages, both on the same Debian Bookworm base. The host tests run in the test stage with
# Bookworm's default GCC 12 and gcovr 7.0; GCC 14 is not in Bookworm's repositories, so this stage
# no longer matches the Ubuntu 24.04 GCC 14 toolchain the CI coverage job uses. The Windows package
# stays on Bookworm's MSVCRT MinGW toolchain, so the DLLs are built with the same compiler and SDK as
# before. Keep the base digest and the pins identical in both stages.

FROM debian:bookworm-slim@sha256:5ae3c39ebd15e229dcedd5cee596b2497182493d41ff162e824ba13fc1b2b867 AS test

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3 \
    python3-protobuf \
    python3-venv \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/gcovr \
    && /opt/gcovr/bin/pip install --no-cache-dir gcovr==7.0

ENV PATH=/opt/gcovr/bin:$PATH

WORKDIR /project
COPY . .
RUN make -C j2534_driver -j2 test && touch /opendiag-host-tests.passed

FROM debian:bookworm-slim@sha256:5ae3c39ebd15e229dcedd5cee596b2497182493d41ff162e824ba13fc1b2b867 AS package

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3 \
    python3-protobuf \
    protobuf-compiler \
    gcc-mingw-w64-i686-win32=12.2.0-14+deb12u1+25.2+b1 \
    g++-mingw-w64-i686-win32=12.2.0-14+deb12u1+25.2+b1 \
    gcc-mingw-w64-i686-win32-runtime=12.2.0-14+deb12u1+25.2+b1 \
    mingw-w64-i686-dev=10.0.0-3 \
    mingw-w64-common=10.0.0-3 \
    binutils-mingw-w64-i686=2.40-2+10.4 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /project
# The package stage starts only after the host tests in the test stage have passed.
COPY --from=test /opendiag-host-tests.passed /opendiag-host-tests.passed
COPY . .
