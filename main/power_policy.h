/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @file power_policy.h
 * @brief When the adapter may go to sleep, and when it should stay up.
 *
 * Pure decisions, no hardware: power.c samples the board once a second and
 * asks. An adapter left plugged into a parked vehicle must not flatten its
 * battery, and must never sleep while anybody is using it.
 */

/** Below this the adapter is on USB power alone: there is no vehicle. */
#define POWER_NO_VEHICLE_MV 6000

typedef struct {
    bool enabled;    /**< Sleep at all. */
    uint32_t idle_s; /**< Quiet time before sleeping. */
    int32_t run_mv;  /**< At or above: engine running, charging. */
} power_cfg_t;

/** One look at the board. */
typedef struct {
    bool client;      /**< Someone connected: a link, USB host, BLE. */
    bool busy;        /**< A bus or a pin is claimed. */
    bool bus_traffic; /**< Edges seen on a CAN receiver since the last look. */
    int32_t vbatt_mv;
} power_sample_t;

/** Why the adapter is staying up, for the shell. */
typedef enum {
    POWER_HOLD_NONE = 0, /**< Counting down: it will sleep. */
    POWER_HOLD_DISABLED,
    POWER_HOLD_NO_VEHICLE,
    POWER_HOLD_RUNNING,
    POWER_HOLD_CLIENT,
    POWER_HOLD_BUSY,
    POWER_HOLD_TRAFFIC,
} power_hold_t;

typedef struct {
    uint32_t quiet_s;  /**< Seconds without a reason to stay up. */
    power_hold_t hold; /**< The reason, on the last tick. */
} power_state_t;

/**
 * @brief Account for @p dt_s seconds described by @p s.
 * @return true when the adapter should sleep now.
 */
bool power_policy_tick(power_state_t *st, const power_cfg_t *cfg,
                       const power_sample_t *s, uint32_t dt_s);

/**
 * @brief After a timer wake-up, before anything else starts.
 * @return true to go straight back to sleep.
 */
bool power_policy_back_to_sleep(const power_cfg_t *cfg, int32_t vbatt_mv);

const char *power_hold_name(power_hold_t h);
