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
/**
 * @brief The bit rate the adapter runs for @p rate, or 0 if it offers none.
 *
 * One list for every front-end. A request within 0.5% of an entry gets that
 * entry, which is what lets 500 kbit/s divided by a whole number (83.3k,
 * 33.3k) or the SJA1000 register pair SLCAN's 's' carries land on it. Anything
 * else is refused rather than run at a nearby speed on a live bus.
 *
 * Beyond the OBD-II rates: 10k and 20k (old body buses), 33.3k, 83.3k and
 * 95.2k (comfort and medium speed buses of some makes), 100k and 800k.
 */
static inline uint32_t can_bitrate_canonical(uint32_t rate) {
    static const uint32_t rates[] = {10000,  20000,  33333,  50000,
                                     83333,  95238,  100000, 125000,
                                     250000, 500000, 800000, 1000000};

    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        if ((uint64_t)rate * 1000 >= (uint64_t)rates[i] * 995 &&
            (uint64_t)rate * 1000 <= (uint64_t)rates[i] * 1005)
            return rates[i];
    }
    return 0;
}

struct can_frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};
