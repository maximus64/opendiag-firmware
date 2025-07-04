/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdint.h>

#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */

#define CAN_EFF_MASK 0x1FFFFFFFU
#define CAN_SFF_MASK 0x000007FFU

struct can_frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};
