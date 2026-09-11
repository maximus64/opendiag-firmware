// SPDX-License-Identifier: GPL-3.0-only
#include "../include/j2534_0404.h"
#include <stddef.h>
_Static_assert(sizeof(J2534_ULONG) == 4, "32-bit scalar");
_Static_assert(sizeof(J2534_LONG) == 4, "32-bit status");
_Static_assert(sizeof(PASSTHRU_MSG) == 4152, "inline message layout");
_Static_assert(offsetof(PASSTHRU_MSG, Data) == 24, "data offset");
_Static_assert(sizeof(SCONFIG) == 8, "config layout");
_Static_assert(sizeof(SCONFIG_LIST) == 4 + sizeof(void *), "config list layout");
_Static_assert(sizeof(SBYTE_ARRAY) == 4 + sizeof(void *), "byte array layout");
