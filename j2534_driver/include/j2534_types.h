// SPDX-License-Identifier: GPL-3.0-only
#ifndef OPENDIAG_J2534_TYPES_H
#define OPENDIAG_J2534_TYPES_H

#include <stdint.h>

#ifdef _WIN32
#if defined(_WIN64)
#error OpenDIAG supports the 32-bit Windows J2534 ABI only
#endif
typedef unsigned long J2534_ULONG;
typedef long J2534_LONG;
#define J2534_CALL __stdcall
#define J2534_EXPORT
#define J2534_PRIu "lu"
#define J2534_PRIx "lx"
#define J2534_PRIX "lX"
#define J2534_PRId "ld"
#else
typedef uint32_t J2534_ULONG;
typedef int32_t J2534_LONG;
#define J2534_CALL
#define J2534_EXPORT __attribute__((visibility("default")))
#define J2534_PRIu "u"
#define J2534_PRIx "x"
#define J2534_PRIX "X"
#define J2534_PRId "d"
#endif

typedef char
    J2534_scalar_width_check[(sizeof(J2534_LONG) == 4 && sizeof(J2534_ULONG) == 4) ? 1 : -1];

#endif
