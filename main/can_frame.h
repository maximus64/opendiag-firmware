/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdint.h>

#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */

#define CAN_EFF_MASK 0x1FFFFFFFU
#define CAN_SFF_MASK 0x000007FFU

/**
 * @brief One classic CAN frame, laid out as SocketCAN lays it out.
 *
 * Eight data bytes is the protocol's own ceiling - ISO 11898-1 - and this
 * chip's TWAI controller is classic only, so it is a hardware ceiling too.
 * CAN FD would be a separate type with its own length field, the way
 * SocketCAN keeps canfd_frame apart from can_frame, rather than a widening
 * of this one.
 *
 * @c id carries CAN_EFF_FLAG and CAN_RTR_FLAG in its top bits, so a frame
 * needs nothing alongside it to be understood. That is what lets it cross
 * bus.h's interface as a bus_msg_t id and payload, with no format in between.
 */
struct can_frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};
