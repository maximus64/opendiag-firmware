/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "can_frame.h"
#include "comm_iface.h"

/** Sessions live for the lifetime of the firmware, so the pool is small. */
#define VIF_MAX_SESSIONS 4
#define VIF_NAME_LEN     8

/** Protocol grammars the firmware carries. Registered once, never removed. */
#define VIF_MAX_FRONTENDS 4

/** Returned by the transfer calls when the session does not hold the bus. */
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
 * @brief Bus parameters.
 *
 * Only CAN has anything to configure today. Pass NULL for the byte buses.
 */
typedef struct {
    uint32_t bitrate; /**< CAN bit rate in bits/s. */
} vif_bus_cfg_t;

typedef enum {
    VIF_PIN_OFF,     /**< Released. */
    VIF_PIN_VOLTAGE, /**< Driven from the boost converter (high side pins). */
    VIF_PIN_GROUND,  /**< Pulled to ground (low side pin). */
} vif_pin_mode_t;

/**
 * @brief A protocol front-end: the grammar half of a session.
 *
 * Everything a front-end needs from the hardware goes through the session
 * handle, so switching one for another inside a session leaves the claims
 * standing. That is the point of the vtable, and the reason a client can raise
 * a programming voltage under ELM327 and then flash over SLCAN without the
 * voltage dropping in between.
 *
 * @p poll is optional. A front-end that only answers what it is asked leaves
 * it NULL and the session task sleeps until bytes arrive; one that pushes
 * unsolicited output - SLCAN forwarding received CAN frames - implements it
 * and is called regularly.
 */
typedef struct {
    const char *name;
    /** Instantiate. @p port is the transport this session speaks on.
     *  Returns the context handed back to the other three calls, or NULL. */
    void *(*create)(vif_session_t *s, comm_port_id_t port);
    void  (*feed)(void *ctx, const uint8_t *data, size_t len);
    void  (*poll)(void *ctx);
    void  (*destroy)(void *ctx);
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
 * @param name Short name for logs and the shell, truncated to VIF_NAME_LEN-1.
 * @param port comm_iface port this session reads and writes, or
 *             COMM_INVALID_PORT_ID for a session with no transport of its
 *             own - the debug shell, which holds claims but is driven from
 *             the console.
 * @return The session, or NULL when the pool is full.
 */
vif_session_t *vif_session_open(const char *name, comm_port_id_t port);

/** @brief Release every claim the session holds and return it to the pool. */
void vif_session_close(vif_session_t *s);

/**
 * @brief Install a front-end, replacing whatever is running.
 *
 * The swap itself happens in the session's own task, so destroy() and
 * create() never run while feed() is mid-command. Passing NULL leaves the
 * session with no grammar; its claims are untouched either way.
 *
 * A session with a port starts its task on the first front-end installed.
 */
esp_err_t vif_session_set_frontend(vif_session_t *s, const vif_frontend_t *fe);

/** @brief The session's name, which is also the link's name. NULL if closed. */
const char *vif_session_name(const vif_session_t *s);

/**
 * @brief True when this session carries a transport.
 *
 * A session without one - the debug shell - holds claims so its hardware
 * commands are arbitrated like any client's, but it has no bytes to parse and
 * no client to answer. A grammar installed on it would never be fed, never be
 * polled, and would hold an instance of that front-end's pool for nothing, so
 * the control plane refuses to put one there.
 */
bool vif_session_is_link(const vif_session_t *s);

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
 * @brief Report that this session's client has gone away.
 *
 * Reverts the grammar to the link default and leaves every claim standing.
 * That split is deliberate: a programming voltage has to survive a replugged
 * cable, but a parser the next client did not ask for must not.
 *
 * Called on the falling edge - a closed port, a dropped BLE connection - and
 * never on connect, because the mode is usually set from the shell *before*
 * the client that needs it opens the port.
 *
 * A no-op when the current grammar is already the default, so a tool that
 * merely toggles DTR does not have its front-end rebuilt underneath it.
 */
void vif_session_link_down(vif_session_t *s);

/** @brief Find an open session by name, for the shell. NULL if there is none. */
vif_session_t *vif_session_find(const char *name);

/** @brief Walk the session pool. NULL for a slot that is not in use. */
vif_session_t *vif_session_at(size_t idx);

/* ------------------------------------------------------------------ *
 * Buses
 * ------------------------------------------------------------------ */

/**
 * @brief Claim the bus and bring its driver up.
 *
 * A session that already has a bus open has it closed first, so switching
 * protocols is one call. Fails with ESP_ERR_INVALID_STATE when another
 * session holds the bus, and with whatever the driver reported if it refused
 * to start - in which case the session is left holding no bus at all, because
 * the previous one is already down by then.
 */
esp_err_t vif_bus_open(vif_session_t *s, vif_bus_t bus, const vif_bus_cfg_t *cfg);

/** @brief Tear the session's bus down and release the claim. */
esp_err_t vif_bus_close(vif_session_t *s);

/** @brief The bus this session holds, or VIF_BUS_NONE. */
vif_bus_t vif_bus_current(const vif_session_t *s);

/** @brief Human readable bus name, for logs and control answers. */
const char *vif_bus_name(vif_bus_t bus);

/**
 * @brief What is live on the wire, and which session holds it.
 *
 * This is what makes a refused claim diagnosable: without it a client that
 * cannot open the bus only learns that it cannot, not who has it.
 */
typedef struct {
    vif_bus_t bus;                /**< VIF_BUS_NONE when nothing is live. */
    uint32_t bitrate;             /**< CAN bit rate; 0 for the byte buses. */
    char owner[VIF_NAME_LEN];     /**< Session name, empty when unheld. */
} vif_bus_info_t;

void vif_bus_info(vif_bus_info_t *out);

/** @brief 0 on success, negative on failure or without the CAN claim. */
int vif_can_send(vif_session_t *s, const struct can_frame *f);

/** @brief 0 when a frame was received, negative on timeout or no claim. */
int vif_can_recv(vif_session_t *s, struct can_frame *f, TickType_t wait);

/** @brief Send on whichever byte bus the session holds. 0 on success. */
int vif_raw_send(vif_session_t *s, const uint8_t *d, size_t len);

/** @brief Receive from the session's byte bus. Bytes read, or negative. */
int vif_raw_recv(vif_session_t *s, uint8_t *d, size_t cap, TickType_t wait);

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
esp_err_t vif_pin_set(vif_session_t *s, int obd_pin, vif_pin_mode_t m, uint32_t mv);

/** @brief Release both pin claims held by @p s. */
esp_err_t vif_pin_release_all(vif_session_t *s);

/** @brief True while any connector pin is energised, whoever holds it. */
bool vif_any_pin_active(void);

int32_t vif_vbatt_mv(void);
int32_t vif_hs_vsense_mv(void);

/** @brief Board identification resistor reading, for factory bring-up. */
uint8_t vif_board_id(void);

/**
 * @brief Run a factory calibration procedure. Prompts on the UART0 console.
 *
 * Drives the high side hardware, so it takes the pin claim for the duration
 * and is refused while another session holds one.
 */
esp_err_t vif_calibrate_hs(vif_session_t *s);
esp_err_t vif_calibrate_vbatt(vif_session_t *s);

/**
 * @brief Ask every session to let go of the hardware and start over.
 *
 * This only raises the request; it touches no driver and takes no time, so it
 * is safe to call from a thread with a small stack - the front panel button's
 * watcher, which is what does call it.
 *
 * The work happens in vif_session_service(), on the thread that owns each
 * session. That is not tidiness: vif_can_recv() and vif_raw_recv() validate
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

/** @brief Print sessions, claims and the live bus to the console. */
void vif_print_debug_info(void);
