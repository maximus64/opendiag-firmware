/* SPDX-License-Identifier: GPL-3.0-only */
#include "power_policy.h"

#include <stddef.h>

/* The first reason found wins; the order is what the shell reports. */
static power_hold_t power_hold_for(const power_cfg_t *cfg,
                                   const power_sample_t *s) {
    if (!cfg->enabled)
        return POWER_HOLD_DISABLED;
    if (s->vbatt_mv < POWER_NO_VEHICLE_MV)
        return POWER_HOLD_NO_VEHICLE;
    if (s->vbatt_mv >= cfg->run_mv)
        return POWER_HOLD_RUNNING;
    if (s->client)
        return POWER_HOLD_CLIENT;
    if (s->busy)
        return POWER_HOLD_BUSY;
    if (s->bus_traffic)
        return POWER_HOLD_TRAFFIC;
    return POWER_HOLD_NONE;
}

bool power_policy_tick(power_state_t *st, const power_cfg_t *cfg,
                       const power_sample_t *s, uint32_t dt_s) {
    st->hold = power_hold_for(cfg, s);
    if (st->hold != POWER_HOLD_NONE) {
        st->quiet_s = 0;
        return false;
    }
    st->quiet_s += dt_s;
    return st->quiet_s >= cfg->idle_s;
}

bool power_policy_back_to_sleep(const power_cfg_t *cfg, int32_t vbatt_mv) {
    /* Only the voltage is known this early. Anything else that would keep
     * the adapter up - traffic, a client - wakes it through its own pin or
     * finds it awake on the next timer. */
    return cfg->enabled && vbatt_mv >= POWER_NO_VEHICLE_MV &&
           vbatt_mv < cfg->run_mv;
}

const char *power_hold_name(power_hold_t h) {
    static const char *const names[] = {
        [POWER_HOLD_NONE] = "nothing (counting down)",
        [POWER_HOLD_DISABLED] = "sleep disabled",
        [POWER_HOLD_NO_VEHICLE] = "no vehicle (USB power)",
        [POWER_HOLD_RUNNING] = "engine running",
        [POWER_HOLD_CLIENT] = "a client is connected",
        [POWER_HOLD_BUSY] = "a bus or pin is in use",
        [POWER_HOLD_TRAFFIC] = "CAN traffic",
    };
    return (unsigned)h < sizeof(names) / sizeof(names[0]) ? names[h] : "?";
}
