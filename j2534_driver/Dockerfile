# Debian Bookworm supplies an MSVCRT MinGW runtime. Keep the compiler and SDK pinned.
FROM debian:bookworm-slim@sha256:5ae3c39ebd15e229dcedd5cee596b2497182493d41ff162e824ba13fc1b2b867

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
COPY . .
