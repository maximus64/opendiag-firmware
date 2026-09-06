/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "bus.h"
#include "comm_iface.h"

/** Protocol grammars the firmware carries. Registered once, never removed. */
#define VIF_MAX_FRONTENDS 4

/** Returned without an open bus claim or from the wrong owner's task. */
#define VIF_ERR_NO_CLAIM (-3)

typedef enum {
    VIF_BUS_NONE = 0,
    VIF_BUS_CAN,
    VIF_BUS_KLINE,
    VIF_BUS_J1850_PWM,
    VIF_BUS_J1850_VPW,
} vif_bus_t;

/**
 * @brief How many buses can be live at once.
 *
 * One per group of buses that share wiring, which is what a claim is really
 * against: CAN, K-Line, and the J1850 pair. PWM and VPW count once between
 * them because they share the transceiver's TX pins and its mode select, so
 * only one modulation can be on the connector at a time. Everything else is
 * independently wired and can run together.
 */
#define VIF_BUS_GROUPS 3

/**
 * @brief Who a claim belongs to.
 *
 * Two, fixed. The link is the one data link, on whichever of USB and BLE has
 * a client; the shell is the debug console on the other USB interface. They
 * are separate owners because they are separate threads that both drive
 * hardware: arbitration keeps the shell from tearing down a bus a client is
 * mid-exchange on, and a driver is only ever torn down by the thread that
 * could be blocked inside it.
 */
typedef enum {
    VIF_OWNER_LINK = 0, /**< The one data link. */
    VIF_OWNER_SHELL,    /**< The debug console. */
    VIF_OWNER_COUNT,
} vif_owner_t;

/** @brief Bus parameters. Only CAN has any; pass NULL for the byte buses. */
typedef bus_cfg_t vif_bus_cfg_t;

typedef enum {
    VIF_PIN_OFF,     /**< Released. */
    VIF_PIN_VOLTAGE, /**< Driven from the boost converter (high side pins). */
    VIF_PIN_GROUND,  /**< Pulled to ground (low side pin). */
} vif_pin_mode_t;

/**
 * @brief A protocol front-end: the grammar the link speaks.
 *
 * One instance, no context handle. The link is singular and runs one grammar
 * at a time, so a front-end is a module with state rather than an object with
 * copies, and there is nothing to hand out twice.
 *
 * Hardware goes through VIF_OWNER_LINK; the transport goes through
 * vif_link_read() and friends, which follow the link as its client moves
 * between USB and BLE.
 *
 * Switching front-ends releases every claim the link holds. @p poll is
 * optional; NULL means sleep until bytes arrive.
 */
typedef struct {
    const char *name;
    /** Bring the grammar up. False when it refused. */
    bool (*start)(void);
    void (*feed)(const uint8_t *data, size_t len);
    void (*poll)(void);
    void (*stop)(void);
} vif_frontend_t;

/**
 * @brief Announce a front-end, so that it can be found by name.
 *
 * Front-ends register themselves at startup, because vif sits below them and
 * must not know what they are. This table is what lets a client name one in a
 * control command, and what the identity string's mode list enumerates.
 *
 * Registering the same vtable twice is harmless. A different one under a name
 * already taken is refused with ESP_ERR_INVALID_STATE.
 */
esp_err_t vif_frontend_register(const vif_frontend_t *fe);

/** @brief Find a registered front-end by name, case insensitively. */
const vif_frontend_t *vif_frontend_find(const char *name);

/** @brief Walk the registered front-ends. NULL once @p idx is past the end. */
const vif_frontend_t *vif_frontend_at(size_t idx);

/**
 * @brief Bring up the board and the arbitration state. Call once, first.
 *
 * Takes over what board_setup() used to be called for: after this, board.h
 * belongs to vif.c alone.
 */
esp_err_t vif_init(void);

/* ------------------------------------------------------------------ *
 * Owners
 * ------------------------------------------------------------------ */

/** @brief Name for the shell, the control plane and the logs. */
const char *vif_owner_name(vif_owner_t owner);

/**
 * @brief Claim the calling task as the shell's thread.
 *
 * The shell has no task of vif's making, so it says which one it is. Until it
 * does, its hardware commands are refused rather than run from an unknown
 * thread. The link's task is recorded by vif_link_start().
 */
void vif_shell_bind(void);

/**
 * @brief Carry out a pending recovery or release on the shell's thread.
 *
 * See vif_recover_all() for why the work happens here rather than where it
 * was asked for.
 *
 * @return true when there was something to do.
 */
bool vif_shell_service(void);

/* ------------------------------------------------------------------ *
 * The link
 * ------------------------------------------------------------------ */

/**
 * @brief Start the link on @p default_fe. Call once, after vif_init().
 *
 * The task runs whether or not a client is attached, so that everything below
 * can assume it exists: it is the one thread allowed inside the link's
 * drivers. @p default_fe is installed on its first pass.
 *
 * The default is what the link falls back to whenever its client detaches, so
 * an off-the-shelf OBD-II app finds what it expects however the link was last
 * used. It is required rather than optional: a link with nothing to fall back
 * to would hand the next client the last one's half-finished conversation.
 */
esp_err_t vif_link_start(const vif_frontend_t *default_fe);

/**
 * @brief Carry out pending link work: a recovery, a release, a grammar swap.
 *
 * The link task's own first step, so nothing else in the firmware calls it. It
 * is public so that a host test can stand in for that task.
 *
 * @return true when there was something to do.
 */
bool vif_link_service(void);

/**
 * @brief Install a front-end, replacing whatever is running.
 *
 * The swap happens in the link's own task, so stop() and start() never run
 * while feed() is mid-command.
 *
 * Every claim the link holds - buses and pins - is released on the way
 * through: a grammar change is a fresh start, and the settings the last one
 * left on a bus mean nothing to the next. A failed bus close delays the
 * replacement until teardown can complete.
 */
void vif_link_set_frontend(const vif_frontend_t *fe);

/** @brief Name of the front-end currently installed, or NULL. */
const char *vif_link_frontend_name(void);

/** @brief The front-end the link reverts to. Set by vif_link_start(). */
const vif_frontend_t *vif_link_default_frontend(void);

/** @brief The transport the link speaks on, or COMM_INVALID_PORT_ID. */
comm_port_id_t vif_link_port(void);

/**
 * @brief Point the link at a transport, or at none.
 *
 * The link follows whichever of USB and BLE has a client on it. Front-ends
 * reach the transport through the calls below on every use, so a rebind needs
 * nothing of them. Pass COMM_INVALID_PORT_ID when the client leaves.
 */
void vif_link_set_port(comm_port_id_t port);

/**
 * @brief Client disconnected. Reverts grammar, releases pins and buses.
 *
 * Called on port close / BLE drop. Never on connect.
 */
void vif_link_down(void);

/* Transport, for front-ends. Each follows the link's current port, so none of
 * them may cache it. */
size_t vif_link_read(uint8_t *buf, size_t len, TickType_t wait);
void vif_link_write(const void *data, size_t len);
void vif_link_flush(void);

/* ------------------------------------------------------------------ *
 * Buses
 * ------------------------------------------------------------------ *
 *
 * All four buses are reached through one interface, bus.h, so a protocol
 * front-end never learns which of them it holds. That is what lets ELM327 and
 * the coming J2534 Pass-Thru layer share a driver: J2534's SET_CONFIG and
 * PassThruIoctl land on vif_bus_param_set() and vif_bus_ioctl() with the
 * identifiers unchanged, and the AT commands that mean the same thing land on
 * the same two calls.
 *
 * Driver operations require the owner's own task. Cross-task release is
 * deferred until that task services it.
 *
 * An owner may hold several buses at once, one per group of shared wiring -
 * see VIF_BUS_GROUPS. Every call below therefore names the bus it means,
 * rather than "the" bus, which no longer says enough.
 */

/**
 * @brief Claim a bus and bring its driver up.
 *
 * Fails with ESP_ERR_INVALID_STATE when the other owner holds this bus or one
 * wired to the same pins, and with whatever the driver reported if it refused
 * to start. Calls from another task also return ESP_ERR_INVALID_STATE.
 *
 * Asking again for a bus in a group this owner already holds swaps them: the
 * old one goes down first, because the two cannot be live together. The
 * owner's other groups are left alone.
 */
esp_err_t vif_bus_open(vif_owner_t owner, vif_bus_t bus,
                       const vif_bus_cfg_t *cfg);

/** Close the driver; retain the claim if it fails. Retry after failure. */
esp_err_t vif_bus_close(vif_owner_t owner, vif_bus_t bus);

/** Release buses, retaining failed claims; foreign tasks queue a request. */
esp_err_t vif_bus_release_all(vif_owner_t owner);

/**
 * @brief What this owner holds in @p bus 's group.
 *
 * Answers with the bus itself when the owner holds exactly that one, with the
 * other member of the group when it holds that instead - PWM against a VPW
 * claim - and VIF_BUS_NONE when it holds neither.
 */
vif_bus_t vif_bus_current(vif_owner_t owner, vif_bus_t bus);

/** Successful open with no teardown pending; retained claims return false. */
bool vif_bus_is_open(vif_owner_t owner, vif_bus_t bus);

/** @brief Human readable bus name, for logs and control answers. */
const char *vif_bus_name(vif_bus_t bus);

/**
 * @brief One live claim, as reported by vif_bus_info().
 *
 * This is what makes a refused claim diagnosable: without it a client that
 * cannot open a bus only learns that it cannot, not who has it.
 */
typedef struct {
    vif_bus_t bus;     /**< VIF_BUS_NONE when the group is idle. */
    uint32_t bitrate;  /**< CAN bit rate; 0 for the byte buses. */
    vif_owner_t owner; /**< Only meaningful when the group is held. */
} vif_bus_claim_t;

/** @brief Snapshot every group's claim. Idle groups come back VIF_BUS_NONE. */
void vif_bus_info(vif_bus_claim_t out[VIF_BUS_GROUPS]);

/**
 * @brief Send on a bus the owner holds.
 *
 * @param msg   Prepared with bus_msg_tx(), which takes the CAN id a frame
 *              needs and leaves it zero for the byte buses.
 * @param flags Per-message BUS_TX_* bits, or 0.
 * @return 0 on success, VIF_ERR_NO_CLAIM without the claim, else BUS_ERR_*.
 */
int vif_bus_send(vif_owner_t owner, vif_bus_t bus, const bus_msg_t *msg,
                 uint32_t flags);

/**
 * @brief Receive from a bus the owner holds.
 *
 * @param msg Prepared with bus_msg_init(). Filled with the message, its
 *            timestamp and its status flags.
 * @return Bytes received, or negative.
 */
int vif_bus_recv(vif_owner_t owner, vif_bus_t bus, bus_msg_t *msg,
                 TickType_t wait);

/**
 * @brief Set one bus parameter. J2534 SET_CONFIG, and the AT commands that
 *        mean the same thing.
 *
 * @return 0, VIF_ERR_NO_CLAIM, or BUS_ERR_UNSUPPORTED when the bus in use
 *         has no such parameter - which is how one front-end can offer the
 *         whole set without knowing which bus it is talking to.
 */
int vif_bus_param_set(vif_owner_t owner, vif_bus_t bus, bus_param_t p,
                      uint32_t value);

/** @brief Read one bus parameter back. */
int vif_bus_param_get(vif_owner_t owner, vif_bus_t bus, bus_param_t p,
                      uint32_t *out);

/**
 * @brief Run a bus operation that is neither a get nor a set.
 *
 * The initialisation sequences, the queue clears, the periodic message. The
 * types @p in and @p out point at are named against each value in
 * bus_ioctl_t.
 */
int vif_bus_ioctl(vif_owner_t owner, vif_bus_t bus, bus_ioctl_t id,
                  const void *in, void *out);

/**
 * @brief A bus's counters.
 *
 * Readable without the claim, and readable for a bus that is down: the
 * counters outlive the driver, and looking at why a bus failed is exactly
 * when nobody holds it.
 */
int vif_bus_stats(vif_bus_t bus, bus_stats_t *out);
int vif_bus_reset_stats(vif_bus_t bus);

/* ------------------------------------------------------------------ *
 * Connector pins and analog
 * ------------------------------------------------------------------ */

/**
 * @brief Drive an OBD-II connector pin, or release it.
 *
 * @param obd_pin Connector pin number: 6, 9, 11-14 high side, 15 low side.
 * @param m       VIF_PIN_VOLTAGE on a high side pin, VIF_PIN_GROUND on the low
 *                side pin, VIF_PIN_OFF on either.
 * @param mv      Millivolts, for VIF_PIN_VOLTAGE.
 *
 * Releasing a pin nobody drives succeeds and does nothing. Anything that would
 * break the one-high-one-low rule, or touch a pin the other owner claimed,
 * fails with ESP_ERR_INVALID_STATE.
 */
esp_err_t vif_pin_set(vif_owner_t owner, int obd_pin, vif_pin_mode_t m,
                      uint32_t mv);

/** @brief Release both pin claims held by @p owner. */
esp_err_t vif_pin_release_all(vif_owner_t owner);

/** @brief True while any connector pin is energised, whoever holds it. */
bool vif_any_pin_active(void);

int32_t vif_vbatt_mv(void);
int32_t vif_hs_vsense_mv(void);

/**
 * @brief Run a factory calibration procedure. Prompts on the UART0 console.
 *
 * Drives the high side hardware, so it takes the pin claim for the duration
 * and is refused while the other owner holds one.
 */
esp_err_t vif_calibrate_hs(vif_owner_t owner);
esp_err_t vif_calibrate_vbatt(void);

/**
 * @brief Write one calibration constant by hand, skipping the procedure above.
 *
 * For restoring a known good set onto a board whose NVS was erased. Drives
 * nothing, but is refused while a connector pin is energised, since the
 * constants decide what the next voltage request does.
 *
 * @param name  Field name, as printed by board_print_info().
 * @param value Raw value, scaled by 10000.
 */
esp_err_t vif_calibration_set(const char *name, int32_t value);

/**
 * @brief Ask both owners to let go of the hardware and start over.
 *
 * This only raises the request; it touches no driver and takes no time, so it
 * is safe to call from a thread with a small stack - the front panel button's
 * watcher, which is what does call it.
 *
 * The work happens on the thread that owns each side: the link's own task,
 * and vif_shell_service() for the console. That is not tidiness - the
 * transfer calls validate the claim, drop the lock and then block inside the
 * driver, so tearing that driver down from another thread would free a queue
 * somebody is waiting on.
 *
 * Recovery is therefore prompt rather than instant - an owner parked on a
 * receive acts when that receive times out. A thread that never comes back is
 * beyond this, and beyond any safe alternative; that is what the reset line is
 * for.
 */
void vif_recover_all(void);

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

/** @brief Print the owners, their claims and the live buses to the console. */
void vif_print_debug_info(void);
