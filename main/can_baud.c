/* SPDX-License-Identifier: GPL-3.0-only */
#include "can_baud.h"

const uint32_t *can_baud_order(size_t *count) {
    /* OBD-II first, then body and comfort buses, then the rare ones. */
    static const uint32_t order[] = {500000,  250000, 125000, 50000,
                                     1000000, 100000, 83333,  95238,
                                     33333,   20000,  10000,  800000};

    *count = sizeof(order) / sizeof(order[0]);
    return order;
}

int can_baud_pick(const can_baud_sample_t *s, size_t n) {
    int best = -1;

    for (size_t i = 0; i < n; i++) {
        if (s[i].frames == 0 || s[i].errors * 4 >= s[i].frames)
            continue;
        if (best < 0 || s[i].frames > s[best].frames)
            best = (int)i;
    }
    return best;
}
