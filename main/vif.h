/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "bus.h"
#include "comm_iface.h"

/** Sessions live for the lifetime of the firmware, so the pool is small. */
#define VIF_MAX_SESSIONS 4
#define VIF_NAME_LEN 8

/** Protocol grammars the firmware carries. Registered once, never removed. */
#define VIF_MAX_FRONTENDS 4

/** Returned without an open bus claim or from the wrong session task. */
#define VIF_ERR_NO_CLAIM (-3)

typedef struct vif_session vif_session_t;

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

/** @brief Bus parameters. Only CAN has any; pass NULL for the byte buses. */
typedef bus_cfg_t vif_bus_cfg_t;

typedef enum {
    VIF_PIN_OFF,     /**< Released. */
    VIF_PIN_VOLTAGE, /**< Driven from the boost converter (high side pins). */
    VIF_PIN_GROUND,  /**< Pulled to ground (low side pin). */
} vif_pin_mode_t;

/**
 * @brief What a session is for.
 *
 * A link speaks a protocol to a client over a transport. The shell holds
 * claims so its hardware commands are arbitrated like any client's, but it
 * has no grammar and no client to answer, so the control plane refuses to
 * put one on it.
 *
 * This is fixed when the session opens rather than read off its transport: a
 * link with nothing connected is still a link, and the shell can set its
 * grammar before the tool that needs it attaches.
 */
typedef enum {
    VIF_SESSION_LOCAL = 0, /**< The debug shell. */
    VIF_SESSION_LINK,      /**< A data link. */
} vif_session_kind_t;

/**
 * @brief A protocol front-end: the grammar half of a session.
 *
 * Hardware access goes through the session handle, and so does the transport:
 * a front-end asks vif_session_port() when it needs one rather than keeping
 * the answer, because the link's transport changes underneath it when a
 * client arrives on the other one.
 *
 * Switching front-ends releases every claim the session holds. @p poll is
 * optional; NULL means sleep until bytes arrive.
 */
typedef struct {
    const char *name;
    /** Instantiate. Returns the context handed back to the other three
     *  calls, or NULL. */
    void *(*create)(vif_session_t *s);
    void (*feed)(void *ctx, const uint8_t *data, size_t len);
    void (*poll)(void *ctx);
    void (*destroy)(void *ctx);
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
 * Sessions
 * ------------------------------------------------------------------ */

/**
 * @brief Open a session, from a static pool. No allocation.
 *
 * Opens without a transport; a link gets one from vif_session_set_port() when
 * a client connects.
 *
 * @param name Short name for logs and the shell, truncated to VIF_NAME_LEN-1.
 * @param kind Link or shell. See vif_session_kind_t.
 * @return The session, or NULL when the pool is full.
 */
vif_session_t *vif_session_open(const char *name, vif_session_kind_t kind);

/** Free after claims close; failures keep the slot. Taskless: creator only. */
void vif_session_close(vif_session_t *s);

/**
 * @brief Install a front-end, replacing whatever is running.
 *
 * The swap itself happens in the session's own task, so destroy() and
 * create() never run while feed() is mid-command. Passing NULL leaves the
 * session with no grammar.
 *
 * Every claim the session holds - buses and pins - is released on the way
 * through: a grammar change is a fresh start, and the settings the last one
 * left on a bus mean nothing to the next.
 *
 * A link starts its task on the first front-end installed, transport or not.
 * Before that, only its creator may install a front-end.
 * A failed bus close delays the replacement until teardown can complete.
 */
esp_err_t vif_session_set_frontend(vif_session_t *s, const vif_frontend_t *fe);

/** @brief The session's name, which is also the link's name. NULL if closed. */
const char *vif_session_name(const vif_session_t *s);

/** @brief True when this session carries a protocol. See vif_session_kind_t. */
bool vif_session_is_link(const vif_session_t *s);

/** @brief The transport this session speaks on, or COMM_INVALID_PORT_ID. */
comm_port_id_t vif_session_port(const vif_session_t *s);

/**
 * @brief Point the session at a transport, or at none.
 *
 * The one link follows whichever of USB and BLE has a client on it, so its
 * transport is not fixed when it opens. Front-ends read it through
 * vif_session_port() on every use, so a rebind needs nothing of them.
 *
 * Pass COMM_INVALID_PORT_ID when the client leaves. Starts the session's task
 * if a grammar is already installed and it has none yet.
 */
esp_err_t vif_session_set_port(vif_session_t *s, comm_port_id_t port);

/** @brief Name of the front-end currently installed, or NULL. */
const char *vif_session_frontend_name(const vif_session_t *s);

/**
 * @brief Set the grammar this link falls back to when its client detaches.
 *
 * Every data link defaults to ELM327, so an off-the-shelf OBD-II app finds
 * what it expects however the link was last used. Setting a default does not
 * install it; main.c does both when it brings the link up.
 */
esp_err_t vif_session_set_default_frontend(vif_session_t *s,
                                           const vif_frontend_t *fe);

/** @brief The front-end this session reverts to, or NULL if it has none. */
const vif_frontend_t *vif_session_default_frontend(const vif_session_t *s);

/**
 * @brief Client disconnected. Reverts grammar, releases pins and buses.
 *
 * Called on port close / BLE drop. Never on connect.
 */
void vif_session_link_down(vif_session_t *s);

/** @brief Find an open session by name, for the shell. NULL if there is none.
 */
vif_session_t *vif_session_find(const char *name);

/** @brief Walk the session pool. NULL for a slot that is not in use. */
vif_session_t *vif_session_at(size_t idx);

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
 * Driver operations require the session task, or its creator before task
 * startup. Cross-task release is deferred until the owner services it.
 *
 * A session may hold several buses at once, one per group of shared wiring -
 * see VIF_BUS_GROUPS. Every call below therefore names the bus it means,
 * rather than "the" bus, which no longer says enough.
 */

/**
 * @brief Claim a bus and bring its driver up.
 *
 * Fails with ESP_ERR_INVALID_STATE when another session holds this bus or one
 * wired to the same pins, and with whatever the driver reported if it refused
 * to start. Calls from another task also return ESP_ERR_INVALID_STATE.
 *
 * Asking again for a bus in a group this session already holds swaps them:
 * the old one goes down first, because the two cannot be live together. The
 * session's other groups are left alone.
 */
esp_err_t vif_bus_open(vif_session_t *s, vif_bus_t bus,
                       const vif_bus_cfg_t *cfg);

/** Close the driver; retain the claim if it fails. Retry after failure. */
esp_err_t vif_bus_close(vif_session_t *s, vif_bus_t bus);

/** Release buses, retaining failed claims; foreign tasks queue a request. */
esp_err_t vif_bus_release_all(vif_session_t *s);

/**
 * @brief What this session holds in @p bus 's group.
 *
 * Answers with the bus itself when the session holds exactly that one, with
 * the other member of the group when it holds that instead - PWM against a
 * VPW claim - and VIF_BUS_NONE when it holds neither.
 */
vif_bus_t vif_bus_current(const vif_session_t *s, vif_bus_t bus);

/** Successful open with no teardown pending; retained claims return false. */
bool vif_bus_is_open(const vif_session_t *s, vif_bus_t bus);

/** @brief Human readable bus name, for logs and control answers. */
const char *vif_bus_name(vif_bus_t bus);

/**
 * @brief One live claim, as reported by vif_bus_info().
 *
 * This is what makes a refused claim diagnosable: without it a client that
 * cannot open a bus only learns that it cannot, not who has it.
 */
typedef struct {
    vif_bus_t bus;            /**< VIF_BUS_NONE when the group is idle. */
    uint32_t bitrate;         /**< CAN bit rate; 0 for the byte buses. */
    char owner[VIF_NAME_LEN]; /**< Session name, empty when unheld. */
} vif_bus_claim_t;

/** @brief Snapshot every group's claim. Idle groups come back VIF_BUS_NONE. */
void vif_bus_info(vif_bus_claim_t out[VIF_BUS_GROUPS]);

/**
 * @brief Send on a bus the session holds.
 *
 * @param msg   Prepared with bus_msg_tx(), which takes the CAN id a frame
 *              needs and leaves it zero for the byte buses.
 * @param flags Per-message BUS_TX_* bits, or 0.
 * @return 0 on success, VIF_ERR_NO_CLAIM without the claim, else BUS_ERR_*.
 */
int vif_bus_send(vif_session_t *s, vif_bus_t bus, const bus_msg_t *msg,
                 uint32_t flags);

/**
 * @brief Receive from a bus the session holds.
 *
 * @param msg Prepared with bus_msg_init(). Filled with the message, its
 *            timestamp and its status flags.
 * @return Bytes received, or negative.
 */
int vif_bus_recv(vif_session_t *s, vif_bus_t bus, bus_msg_t *msg,
                 TickType_t wait);

/**
 * @brief Set one bus parameter. J2534 SET_CONFIG, and the AT commands that
 *        mean the same thing.
 *
 * @return 0, VIF_ERR_NO_CLAIM, or BUS_ERR_UNSUPPORTED when the bus in use
 *         has no such parameter - which is how one front-end can offer the
 *         whole set without knowing which bus it is talking to.
 */
int vif_bus_param_set(vif_session_t *s, vif_bus_t bus, bus_param_t p,
                      uint32_t value);

/** @brief Read one bus parameter back. */
int vif_bus_param_get(vif_session_t *s, vif_bus_t bus, bus_param_t p,
                      uint32_t *out);

/**
 * @brief Run a bus operation that is neither a get nor a set.
 *
 * The initialisation sequences, the queue clears, the periodic message. The
 * types @p in and @p out point at are named against each value in
 * bus_ioctl_t.
 */
int vif_bus_ioctl(vif_session_t *s, vif_bus_t bus, bus_ioctl_t id,
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
 * break the one-high-one-low rule, or touch a pin another session claimed,
 * fails with ESP_ERR_INVALID_STATE.
 */
esp_err_t vif_pin_set(vif_session_t *s, int obd_pin, vif_pin_mode_t m,
                      uint32_t mv);

/** @brief Release both pin claims held by @p s. */
esp_err_t vif_pin_release_all(vif_session_t *s);

/** @brief True while any connector pin is energised, whoever holds it. */
bool vif_any_pin_active(void);

int32_t vif_vbatt_mv(void);
int32_t vif_hs_vsense_mv(void);

/**
 * @brief Run a factory calibration procedure. Prompts on the UART0 console.
 *
 * Drives the high side hardware, so it takes the pin claim for the duration
 * and is refused while another session holds one.
 */
esp_err_t vif_calibrate_hs(vif_session_t *s);
esp_err_t vif_calibrate_vbatt(vif_session_t *s);

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
esp_err_t vif_calibration_set(vif_session_t *s, const char *name,
                              int32_t value);

/**
 * @brief Ask every session to let go of the hardware and start over.
 *
 * This only raises the request; it touches no driver and takes no time, so it
 * is safe to call from a thread with a small stack - the front panel button's
 * watcher, which is what does call it.
 *
 * The work happens in vif_session_service(), on the thread that owns each
 * session. That is not tidiness: the transfer calls validate
 * the claim, drop the lock and then block inside the driver, so tearing that
 * driver down from another thread would free a queue somebody is waiting on.
 *
 * Recovery is therefore prompt rather than instant - a session parked on a
 * receive acts when that receive times out. A session whose task never comes
 * back is beyond this, and beyond any safe alternative; that is what the
 * reset line is for.
 */
void vif_recover_all(void);

/**
 * @brief Carry out a pending recovery on a session this thread owns.
 *
 * Releases the session's bus and pin claims and puts its default grammar back.
 * Sessions with a task of their own call this at the top of their loop; the
 * debug shell has no session task, so its own idle poll calls it instead.
 *
 * The rule, and it is the whole point: a session's claims are released by
 * whoever owns its thread, never by whoever asked.
 *
 * @return true when there was something to do.
 */
bool vif_session_service(vif_session_t *s);

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

/** @brief Print sessions, claims and the live buses to the console. */
void vif_print_debug_info(void);
