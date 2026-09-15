// SPDX-License-Identifier: GPL-3.0-only
#include "../include/j2534.h"
#include <stddef.h>
_Static_assert(sizeof(J2534_ULONG) == 4, "32-bit scalar");
_Static_assert(sizeof(J2534_LONG) == 4, "32-bit status");
_Static_assert(sizeof(PASSTHRU_MSG) == 32 + sizeof(void *), "message layout");
_Static_assert(offsetof(PASSTHRU_MSG, DataBuffer) == 28, "buffer offset");
_Static_assert(sizeof(RESOURCE_STRUCT) == 8 + sizeof(void *), "resource layout");
_Static_assert(sizeof(ISO15765_CHANNEL_DESCRIPTOR) == 18, "descriptor packing");
_Static_assert(sizeof(SDEVICE) == 104, "device layout");
_Static_assert(sizeof(REPEAT_MSG_SETUP) == 8 + 3 * sizeof(PASSTHRU_MSG), "repeat layout");
_Static_assert(offsetof(REPEAT_MSG_SETUP, RepeatMsgData) == 8, "repeat message offset");
_Static_assert(sizeof(SPARAM) == 12, "discovery parameter layout");
_Static_assert(sizeof(SPARAM_LIST) == 4 + sizeof(void *), "discovery list layout");
_Static_assert(offsetof(SPARAM_LIST, ParamPtr) == 4, "discovery pointer offset");
