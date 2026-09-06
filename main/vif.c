/* SPDX-License-Identifier: GPL-3.0-only */
#include "vif.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board.h"
#include "can_bus.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline.h"
#include "ws2812_led.h"

#define TAG "VIF"

/** Idle wait in the link task. Also how long a front-end switch requested
 *  from another task waits before it is applied. */
#define VIF_IDLE_WAIT_MS 50

/** Wait between poll() calls for a front-end that pushes unsolicited output.
 *  Short enough that the CAN driver's receive queue cannot overflow before the
 *  next drain: a saturated 500 kbit/s bus of minimum length frames delivers
 *  about 21 in 2 ms, against a 32 deep queue.
 *  CONFIG_FREERTOS_HZ is 1000, so this really is 2 ms and not a rounding
 *  error - see wait_ticks() for what happens if that ever changes. */
#define VIF_POLL_WAIT_MS 2

/** ms → ticks, floor at 1 to prevent zero-tick busy loops. */
static inline TickType_t wait_ticks(uint32_t ms) {
    TickType_t ticks = pdMS_TO_TICKS(ms);

    return ticks ? ticks : 1;
}

/** Pin the high side calibration procedure drives. */
#define VIF_CAL_HS_PIN 12

/**
 * @brief The groups a claim is really against: shared wiring, not a bus.
 *
 * PWM and VPW land in one slot because they share the transceiver's TX pins
 * and its mode select. The rest are wired apart and run together.
 */
typedef enum {
    VIF_RES_CAN = 0,
    VIF_RES_KLINE,
    VIF_RES_J1850,
    VIF_RES_COUNT,
} vif_res_t;

_Static_assert(VIF_RES_COUNT == VIF_BUS_GROUPS, "group count out of step");

static struct {
    SemaphoreHandle_t lock;
    bool inited;

    /* The one thread allowed inside each owner's drivers. NULL until the link
     * task starts and until the shell says which task it is. */
    TaskHandle_t task[VIF_OWNER_COUNT];

    /* Both consumed by the owning thread: recover_req from the front panel
     * button, release_req from anybody who asked for the claims to go while
     * standing on the wrong thread to take a driver down. */
    volatile bool recover_req[VIF_OWNER_COUNT];
    volatile bool release_req[VIF_OWNER_COUNT];

    /* One live bus per group, and who opened it. owner is only meaningful
     * while bus is set. */
    struct {
        vif_bus_t bus;
        bool open;
        uint32_t bitrate;
        vif_owner_t owner;
    } claim[VIF_RES_COUNT];

    /* SAE J2534: one high side and one low side driver at a time. -1 when the
     * driver is idle; the owner says who may switch it off. */
    int8_t hs_pin;
    int8_t ls_pin;
    vif_owner_t hs_owner;
    vif_owner_t ls_owner;

    struct {
        /* The transport, which a client brings with it and takes away again.
         * Read on every pass of the link task and by every front-end, written
         * under the lock. */
        volatile comm_port_id_t port;

        /* fe is only ever written by the link task once that task exists, so
         * feed() cannot be running against a grammar that stop() has already
         * shut down. */
        const vif_frontend_t *fe;
        const vif_frontend_t *fe_next;
        /* Set under the lock, consumed by the link task, and read outside the
         * lock on the way in - see vif_link_service(). */
        volatile bool fe_switch;

        /* What the link goes back to when its client detaches. */
        const vif_frontend_t *fe_default;
    } link;
} g;

#define LOCK() xSemaphoreTake(g.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g.lock)

static bool vif_owner_service(vif_owner_t owner);

const char *vif_owner_name(vif_owner_t owner) {
    switch (owner) {
    case VIF_OWNER_LINK:
        return "link";
    case VIF_OWNER_SHELL:
        return "shell";
    default:
        return "?";
    }
}

/* ------------------------------------------------------------------ *
 * Drivers
 * ------------------------------------------------------------------ */

/**
 * @brief Every bus behind one interface.
 *
 * can_bus.c, kline.c, j1850_pwm.c and j1850_vpw.c each publish a bus_ops_t,
 * so this is a lookup rather than a switch repeated in every call. Adding a
 * fifth bus is one line here and nothing at all above.
 */
static const bus_ops_t *bus_for(vif_bus_t bus) {
    switch (bus) {
    case VIF_BUS_CAN:
        return &can_bus_ops;
    case VIF_BUS_KLINE:
        return &kline_bus_ops;
    case VIF_BUS_J1850_PWM:
        return &j1850_pwm_bus_ops;
    case VIF_BUS_J1850_VPW:
        return &j1850_vpw_bus_ops;
    default:
        return NULL;
    }
}

/**
 * @brief Is this the thread that could be inside @p owner 's drivers?
 *
 * A transfer checks the claim under the lock and then lets go of it, because
 * a receive parks for as long as its timeout and holding the lock that long
 * would stall the other owner. So while a transfer is in flight, nothing says
 * so - and a close arriving from another thread would free the queue, the
 * channels and the transmit mutex under a task still using them. On a long
 * block transfer that window is seconds wide, and the consequence is a crash
 * in the middle of a programming session.
 *
 * The rule that closes it: a driver is torn down only by the thread that
 * could be blocked in it. Anybody else asks, and the owning thread does it -
 * the link task at the top of its loop, vif_shell_service() for the console.
 */
static bool on_owner_thread(vif_owner_t owner) {
    return owner < VIF_OWNER_COUNT && g.task[owner] &&
           g.task[owner] == xTaskGetCurrentTaskHandle();
}

/** @brief The wiring @p bus claims, or VIF_RES_COUNT if it is not a bus. */
static vif_res_t bus_group(vif_bus_t bus) {
    switch (bus) {
    case VIF_BUS_CAN:
        return VIF_RES_CAN;
    case VIF_BUS_KLINE:
        return VIF_RES_KLINE;
    case VIF_BUS_J1850_PWM:
    case VIF_BUS_J1850_VPW:
        return VIF_RES_J1850;
    default:
        return VIF_RES_COUNT;
    }
}

const char *vif_bus_name(vif_bus_t bus) {
    switch (bus) {
    case VIF_BUS_CAN:
        return "CAN";
    case VIF_BUS_KLINE:
        return "K-Line";
    case VIF_BUS_J1850_PWM:
        return "J1850 PWM";
    case VIF_BUS_J1850_VPW:
        return "J1850 VPW";
    default:
        return "none";
    }
}

static esp_err_t bus_driver_up(vif_bus_t bus, const vif_bus_cfg_t *cfg) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A bus is brought up here but not initialised. Selecting a protocol
     * and establishing a session are two different things on K-Line - the
     * 5 baud handshake takes upwards of two and a half seconds and can fail
     * on a vehicle that is perfectly reachable a moment later - so the client
     * asks for it separately, through vif_bus_ioctl(). It is also what lets
     * the shell watch a bus without touching it. */
    return ops->open(cfg);
}

static esp_err_t bus_driver_down(vif_bus_t bus) {
    const bus_ops_t *ops = bus_for(bus);

    return ops ? ops->close() : ESP_ERR_INVALID_ARG;
}

/* ------------------------------------------------------------------ *
 * Pins
 * ------------------------------------------------------------------ */

/** Reflect pin state on status LED. */
static void led_refresh_locked(void) {
    ws2812_led_set_state((g.hs_pin != -1 || g.ls_pin != -1) ? LED_STATE_PIN_LIVE
                                                            : LED_STATE_IDLE);
}

static void hs_release_locked(void) {
    if (g.hs_pin == -1) {
        return;
    }

    board_set_hs_state((enum hs_pin)g.hs_pin, 0);
    board_set_hs_boost_en(0);
    board_set_hs_voltage(0);

    g.hs_pin = -1;
}

static void ls_release_locked(void) {
    if (g.ls_pin == -1) {
        return;
    }

    board_set_ls_state((enum ls_pin)g.ls_pin, 0);

    g.ls_pin = -1;
}

/* ------------------------------------------------------------------ *
 * The front-end table
 *
 * Front-ends announce themselves; vif does not know their names at compile
 * time, because it sits underneath them. The table is read far more often
 * than it is written - once per registration at startup, then on every
 * lookup by name - and every entry outlives the firmware, so it needs no
 * locking beyond the ordering startup already gives it.
 * ------------------------------------------------------------------ */

static const vif_frontend_t *g_frontends[VIF_MAX_FRONTENDS];
static size_t g_frontend_count;

esp_err_t vif_frontend_register(const vif_frontend_t *fe) {
    if (!fe || !fe->name) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < g_frontend_count; i++) {
        if (g_frontends[i] == fe) {
            return ESP_OK;
        }
        if (strcasecmp(g_frontends[i]->name, fe->name) == 0) {
            ESP_LOGE(TAG, "front-end '%s' is already registered", fe->name);
            return ESP_ERR_INVALID_STATE;
        }
    }

    if (g_frontend_count == VIF_MAX_FRONTENDS) {
        ESP_LOGE(TAG, "no room for front-end '%s'", fe->name);
        return ESP_ERR_NO_MEM;
    }

    g_frontends[g_frontend_count++] = fe;

    ESP_LOGI(TAG, "front-end '%s' registered", fe->name);
    return ESP_OK;
}

const vif_frontend_t *vif_frontend_find(const char *name) {
    if (!name)
        return NULL;

    for (size_t i = 0; i < g_frontend_count; i++) {
        if (strcasecmp(g_frontends[i]->name, name) == 0) {
            return g_frontends[i];
        }
    }

    return NULL;
}

const vif_frontend_t *vif_frontend_at(size_t idx) {
    return idx < g_frontend_count ? g_frontends[idx] : NULL;
}

/* ------------------------------------------------------------------ *
 * The link
 * ------------------------------------------------------------------ */

/** @brief Bring @p fe up. False when its start() refused. */
static bool frontend_start(const vif_frontend_t *fe) {
    if (fe->start && !fe->start()) {
        return false;
    }

    g.link.fe = fe;

    /* Whatever the previous front-end still had to say has been said and
     * dropped by now - its stop() ran before this - so the port is this
     * one's to answer on. Harmless when nothing muted it. */
    comm_port_client_ready(g.link.port);

    ESP_LOGI(TAG, "link: front-end %s", fe->name);
    return true;
}

/** Install @p fe. Old grammar shut down first; previous restored on refusal so
 *  a bad switch doesn't leave the port mute. Releases every claim. */
static bool link_apply_frontend(const vif_frontend_t *fe) {
    const vif_frontend_t *prev = g.link.fe;

    if (prev && prev->stop) {
        prev->stop();
    }
    g.link.fe = NULL;

    /* A grammar change is a fresh start. The bus the last one left open was
     * claimed for its protocol, and the timings and filters on it mean
     * nothing to the next; stop() has already had its chance to close the
     * session down politely. */
    vif_pin_release_all(VIF_OWNER_LINK);
    if (vif_bus_release_all(VIF_OWNER_LINK) != ESP_OK) {
        return false;
    }

    if (!fe || frontend_start(fe)) {
        return true;
    }

    ESP_LOGE(TAG, "link: front-end %s refused to start", fe->name);

    if (prev && !frontend_start(prev)) {
        ESP_LOGE(TAG, "link: %s did not come back either", prev->name);
    }
    return true;
}

bool vif_link_service(void) {
    const vif_frontend_t *next = NULL;
    bool switching = false;
    bool did;

    if (!on_owner_thread(VIF_OWNER_LINK)) {
        return false;
    }

    /* Ahead of the switch below, so a default grammar a recovery puts back is
     * installed in the same pass rather than the next one. */
    did = vif_owner_service(VIF_OWNER_LINK);

    /* Read the request under the lock, apply it outside: start() and stop()
     * reach back into vif for their own claims. The unlocked test is the same
     * bargain as the one above - the hot pass takes no lock at all. */
    if (g.link.fe_switch) {
        LOCK();
        if (g.link.fe_switch) {
            g.link.fe_switch = false;
            next = g.link.fe_next;
            switching = true;
        }
        UNLOCK();
    }

    if (!switching) {
        return did;
    }

    if (!link_apply_frontend(next)) {
        /* Teardown could not finish. The link is left with no grammar, so the
         * task's idle wait paces the retry. */
        LOCK();
        if (!g.link.fe_switch) {
            g.link.fe_next = next;
            g.link.fe_switch = true;
        }
        UNLOCK();
    }

    return true;
}

/** @brief Is a link control request still outstanding? */
static bool link_retry_pending(void) {
    return g.link.fe_switch || g.release_req[VIF_OWNER_LINK] ||
           g.recover_req[VIF_OWNER_LINK];
}

static void link_task(void *param) {
    uint8_t buf[64];

    (void)param;

    for (;;) {
        const vif_frontend_t *fe;
        comm_port_id_t port;
        TickType_t wait;
        size_t n;

        vif_link_service();

        fe = g.link.fe;
        port = g.link.port;

        /* Nothing to read and nobody to answer: a link between clients. The
         * read below would not block on a port that does not exist, so the
         * wait has to happen here instead. */
        if (!fe || port == COMM_INVALID_PORT_ID) {
            vTaskDelay(wait_ticks(VIF_IDLE_WAIT_MS));
            continue;
        }

        /* A front-end with unsolicited output has to come round often enough
         * to drain the bus; one without can sleep until a byte shows up, bar
         * the wakeup that lets a front-end switch land. */
        wait = wait_ticks(fe->poll ? VIF_POLL_WAIT_MS : VIF_IDLE_WAIT_MS);

        n = comm_port_read(port, buf, sizeof(buf), wait);
        if (n && fe->feed) {
            fe->feed(buf, n);
        }
        /* feed() may have yielded while another task requested teardown. */
        if (link_retry_pending()) {
            continue;
        }
        if (fe->poll) {
            fe->poll();
        }
    }
}

esp_err_t vif_link_start(const vif_frontend_t *default_fe) {
    TaskHandle_t task = NULL;

    if (!g.inited) {
        ESP_LOGE(TAG, "vif_init() has not run");
        return ESP_ERR_INVALID_STATE;
    }
    if (!default_fe) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g.task[VIF_OWNER_LINK]) {
        return ESP_OK;
    }

    /* Ahead of the task, so the default stands even if it fails to start and
     * the shell is left asking what the link would fall back to. */
    g.link.fe_default = default_fe;
    vif_link_set_frontend(default_fe);

    if (xTaskCreate(link_task, "vif_link", 4 * 1024, NULL, 5, &task) !=
        pdPASS) {
        ESP_LOGE(TAG, "failed to start the link task");
        return ESP_FAIL;
    }

    g.task[VIF_OWNER_LINK] = task;
    return ESP_OK;
}

void vif_shell_bind(void) {
    g.task[VIF_OWNER_SHELL] = xTaskGetCurrentTaskHandle();
}

void vif_link_set_frontend(const vif_frontend_t *fe) {
    /* Handed to the link task, so stop() and start() cannot run underneath a
     * feed() that is mid-command. */
    LOCK();
    g.link.fe_next = fe;
    g.link.fe_switch = true;
    UNLOCK();
}

comm_port_id_t vif_link_port(void) { return g.link.port; }

void vif_link_set_port(comm_port_id_t port) {
    LOCK();
    g.link.port = port;
    UNLOCK();

    if (port == COMM_INVALID_PORT_ID) {
        ESP_LOGI(TAG, "link: transport detached");
        return;
    }

    /* Muted when its last client left; this one may be answered. */
    comm_port_client_ready(port);

    ESP_LOGI(TAG, "link: on port %u", (unsigned)port);
}

size_t vif_link_read(uint8_t *buf, size_t len, TickType_t wait) {
    return comm_port_read(g.link.port, buf, len, wait);
}

void vif_link_write(const void *data, size_t len) {
    comm_port_write(g.link.port, data, len);
}

void vif_link_flush(void) { comm_port_flush(g.link.port); }

const char *vif_link_frontend_name(void) {
    return g.link.fe ? g.link.fe->name : NULL;
}

const vif_frontend_t *vif_link_default_frontend(void) {
    return g.link.fe_default;
}

void vif_link_down(void) {
    const vif_frontend_t *want = g.link.fe_default;

    if (!want) {
        return; /* before vif_link_start(); there is no client to have lost */
    }

    /* Flush stale input and mute the port before the next client arrives. */
    comm_port_client_gone(g.link.port);

    /* Reinstall unconditionally — the departed client's parser state (echo,
     * headers, line buffer) must not reach the next client. */
    ESP_LOGI(TAG, "link: client gone, restarting %s", want->name);

    /*
     * The switch is the whole teardown: it releases every claim, and releases
     * them after the outgoing grammar's stop() has run. That order is the
     * point: a KWP ECU is owed a StopCommunication, and stop() needs the bus
     * still open to send it. Asking for a release here as well would pull the
     * bus out from under that, because the service pass acts on the request
     * before it applies the switch.
     */
    vif_link_set_frontend(want);
}

void vif_bus_info(vif_bus_claim_t out[VIF_BUS_GROUPS]) {
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out) * VIF_BUS_GROUPS);

    LOCK();
    for (int i = 0; i < VIF_RES_COUNT; i++) {
        out[i].bus = g.claim[i].bus;
        out[i].bitrate = g.claim[i].bitrate;
        out[i].owner = g.claim[i].owner;
    }
    UNLOCK();
}

void vif_recover_all(void) {
    /* No logging: the front panel button's watcher calls this, and it is sized
     * on the promise that nothing here reaches vprintf. Each owner announces
     * itself from its own service pass instead. */
    LOCK();
    for (int i = 0; i < VIF_OWNER_COUNT; i++) {
        g.recover_req[i] = true;
    }
    UNLOCK();
}

/**
 * @brief Carry out a pending recovery or release on the owning thread.
 *
 * The rule, and it is the whole point: an owner's claims are released by
 * whoever owns its thread, never by whoever asked.
 */
static bool vif_owner_service(vif_owner_t owner) {
    bool recover;
    bool release;

    if (!on_owner_thread(owner)) {
        return false;
    }

    /*
     * Nothing pending is the answer almost every time, and the link task asks
     * on every pass - up to five hundred a second while a poll-driven grammar
     * drains a bus. So the flags are read unlocked first and the lock is taken
     * only when there is something to take it for. A set that races this read
     * is seen on the next pass, which is the latency a request already waits
     * for; see vif_recover_all().
     */
    if (!g.recover_req[owner] && !g.release_req[owner]) {
        return false;
    }

    LOCK();
    recover = g.recover_req[owner];
    release = g.release_req[owner];
    g.recover_req[owner] = false;
    g.release_req[owner] = false;
    UNLOCK();

    if (!recover && !release) {
        return false;
    }

    if (recover) {
        ESP_LOGW(TAG, "%s: releasing every claim", vif_owner_name(owner));
    }

    /* Safe here and only here: this thread owns the claims, so it is not
     * sitting inside a driver call on the bus about to be torn down. */
    esp_err_t err = vif_bus_release_all(owner);
    vif_pin_release_all(owner);
    if (err != ESP_OK) {
        LOCK();
        g.release_req[owner] = true;
        g.recover_req[owner] |= recover;
        UNLOCK();
        return true;
    }

    /* A recovery puts the grammar back too. A release does not: whoever
     * asked for it has already said what should run next. */
    if (recover && owner == VIF_OWNER_LINK) {
        vif_link_set_frontend(g.link.fe_default);
    }

    return true;
}

bool vif_shell_service(void) { return vif_owner_service(VIF_OWNER_SHELL); }

/* ------------------------------------------------------------------ *
 * Buses
 * ------------------------------------------------------------------ */

esp_err_t vif_bus_open(vif_owner_t owner, vif_bus_t bus,
                       const vif_bus_cfg_t *cfg) {
    vif_res_t grp = bus_group(bus);
    esp_err_t err;

    if (owner >= VIF_OWNER_COUNT || grp == VIF_RES_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!on_owner_thread(owner)) {
        return ESP_ERR_INVALID_STATE;
    }

    LOCK();

    if (g.claim[grp].bus != VIF_BUS_NONE && g.claim[grp].owner != owner) {
        vif_bus_t held = g.claim[grp].bus;
        vif_owner_t by = g.claim[grp].owner;
        UNLOCK();
        ESP_LOGW(TAG, "%s: %s is held by %s", vif_owner_name(owner),
                 vif_bus_name(held), vif_owner_name(by));
        return ESP_ERR_INVALID_STATE;
    }

    vif_bus_t old = g.claim[grp].bus;
    g.claim[grp].open = false;
    if (old == VIF_BUS_NONE) {
        g.claim[grp].bus = bus;
        g.claim[grp].owner = owner;
    }
    UNLOCK();

    /* The reservation excludes the other owner while the driver blocks. */
    if (old != VIF_BUS_NONE) {
        err = bus_driver_down(old);
        if (err != ESP_OK)
            return err;
    }

    LOCK();
    g.claim[grp].bus = bus;
    g.claim[grp].bitrate = cfg ? cfg->bitrate : 0;
    g.claim[grp].owner = owner;
    UNLOCK();

    err = bus_driver_up(bus, cfg);
    esp_err_t cleanup = err == ESP_OK ? ESP_OK : bus_driver_down(bus);

    LOCK();
    if (err == ESP_OK) {
        g.claim[grp].open = true;
    } else if (cleanup == ESP_OK) {
        memset(&g.claim[grp], 0, sizeof(g.claim[grp]));
    }
    UNLOCK();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: %s failed to start: %s", vif_owner_name(owner),
                 vif_bus_name(bus), esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "%s: %s bus open", vif_owner_name(owner), vif_bus_name(bus));
    return ESP_OK;
}

esp_err_t vif_bus_close(vif_owner_t owner, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);

    if (owner >= VIF_OWNER_COUNT || grp == VIF_RES_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!on_owner_thread(owner)) {
        /* Every caller of this closes a bus it holds itself, so reaching
         * here is a mistake rather than a case to handle: releasing on
         * somebody's behalf is vif_bus_release_all()'s job. */
        ESP_LOGE(TAG, "%s: %s closed from the wrong thread",
                 vif_owner_name(owner), vif_bus_name(bus));
        return ESP_ERR_INVALID_STATE;
    }

    LOCK();

    if (g.claim[grp].bus == VIF_BUS_NONE) {
        UNLOCK();
        return ESP_OK;
    }
    if (g.claim[grp].owner != owner) {
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }

    vif_bus_t held = g.claim[grp].bus;
    g.claim[grp].open = false;
    UNLOCK();

    esp_err_t err = bus_driver_down(held);
    if (err == ESP_OK) {
        LOCK();
        memset(&g.claim[grp], 0, sizeof(g.claim[grp]));
        UNLOCK();
    }
    return err;
}

esp_err_t vif_bus_release_all(vif_owner_t owner) {
    esp_err_t result = ESP_OK;

    if (owner >= VIF_OWNER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!on_owner_thread(owner)) {
        /* Prompt rather than instant: the owner acts when its receive times
         * out. That is the price of never freeing a driver underneath it. */
        LOCK();
        g.release_req[owner] = true;
        UNLOCK();
        return ESP_OK;
    }

    for (int i = 0; i < VIF_RES_COUNT; i++) {
        LOCK();
        if (g.claim[i].bus == VIF_BUS_NONE || g.claim[i].owner != owner) {
            UNLOCK();
            continue;
        }
        vif_bus_t held = g.claim[i].bus;
        g.claim[i].open = false;
        UNLOCK();

        esp_err_t err = bus_driver_down(held);
        if (err == ESP_OK) {
            LOCK();
            memset(&g.claim[i], 0, sizeof(g.claim[i]));
            UNLOCK();
        } else {
            result = err;
        }
    }

    return result;
}

vif_bus_t vif_bus_current(vif_owner_t owner, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    vif_bus_t held;

    if (owner >= VIF_OWNER_COUNT || grp == VIF_RES_COUNT) {
        return VIF_BUS_NONE;
    }

    LOCK();
    held = (g.claim[grp].owner == owner) ? g.claim[grp].bus : VIF_BUS_NONE;
    UNLOCK();

    return held;
}

bool vif_bus_is_open(vif_owner_t owner, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    bool open;

    if (owner >= VIF_OWNER_COUNT || grp == VIF_RES_COUNT) {
        return false;
    }

    LOCK();
    open = g.claim[grp].owner == owner && g.claim[grp].bus == bus &&
           g.claim[grp].open;
    UNLOCK();

    return open;
}

/**
 * @brief Confirm @p owner holds @p bus itself, not merely its group.
 *
 * The transfer calls check the claim under the lock and then let go of it: a
 * receive parks for as long as its timeout, and holding the claim lock that
 * long would stall the other owner's commands. Only the holder can close a
 * bus, and each owner is driven by one task, so the claim cannot evaporate
 * underneath its own transfer.
 */
static bool bus_owned_by(vif_owner_t owner, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    bool ok;

    if (owner >= VIF_OWNER_COUNT || grp == VIF_RES_COUNT ||
        !on_owner_thread(owner)) {
        return false;
    }

    LOCK();
    ok = (g.claim[grp].bus == bus && g.claim[grp].owner == owner &&
          g.claim[grp].open);
    UNLOCK();

    return ok;
}

int vif_bus_send(vif_owner_t owner, vif_bus_t bus, const bus_msg_t *msg,
                 uint32_t flags) {
    const bus_ops_t *ops = bus_for(bus);

    if (!msg || !msg->data || !ops || !bus_owned_by(owner, bus)) {
        return VIF_ERR_NO_CLAIM;
    }

    /* What counts as an empty message is the bus's own business: nothing to
     * say on a byte bus, but a legal frame on CAN, where the length code can
     * be zero. Each driver refuses its own. */
    return ops->send(msg, flags);
}

int vif_bus_recv(vif_owner_t owner, vif_bus_t bus, bus_msg_t *msg,
                 TickType_t wait) {
    const bus_ops_t *ops = bus_for(bus);

    if (!msg || !msg->data || !ops || !bus_owned_by(owner, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (msg->cap == 0) {
        return BUS_ERR_BAD_ARG;
    }

    return ops->recv(msg, wait);
}

int vif_bus_param_set(vif_owner_t owner, vif_bus_t bus, bus_param_t p,
                      uint32_t value) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !bus_owned_by(owner, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (!ops->set_param) {
        return BUS_ERR_UNSUPPORTED;
    }

    return ops->set_param(p, value);
}

int vif_bus_param_get(vif_owner_t owner, vif_bus_t bus, bus_param_t p,
                      uint32_t *out) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !out || !bus_owned_by(owner, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (!ops->get_param) {
        return BUS_ERR_UNSUPPORTED;
    }

    return ops->get_param(p, out);
}

int vif_bus_ioctl(vif_owner_t owner, vif_bus_t bus, bus_ioctl_t id,
                  const void *in, void *out) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !bus_owned_by(owner, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (!ops->ioctl) {
        return BUS_ERR_UNSUPPORTED;
    }

    return ops->ioctl(id, in, out);
}

int vif_bus_stats(vif_bus_t bus, bus_stats_t *out) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !out) {
        return BUS_ERR_BAD_ARG;
    }
    if (!ops->get_stats) {
        return BUS_ERR_UNSUPPORTED;
    }

    ops->get_stats(out);
    return 0;
}

int vif_bus_reset_stats(vif_bus_t bus) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops) {
        return BUS_ERR_BAD_ARG;
    }
    if (!ops->reset_stats) {
        return BUS_ERR_UNSUPPORTED;
    }

    ops->reset_stats();
    return 0;
}

esp_err_t vif_pin_set(vif_owner_t owner, int obd_pin, vif_pin_mode_t m,
                      uint32_t mv) {
    bool high_side = (obd_pin == HS_OBD_PIN_6 || obd_pin == HS_OBD_PIN_9 ||
                      obd_pin == HS_OBD_PIN_11 || obd_pin == HS_OBD_PIN_12 ||
                      obd_pin == HS_OBD_PIN_13 || obd_pin == HS_OBD_PIN_14);
    bool low_side = (obd_pin == LS_OBD_PIN_15);

    if (owner >= VIF_OWNER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!high_side && !low_side) {
        return ESP_ERR_INVALID_ARG;
    }
    /* The board drives voltage from the high side switches and ground from the
     * low side one; neither pin group can do the other's job. */
    if ((m == VIF_PIN_VOLTAGE && !high_side) ||
        (m == VIF_PIN_GROUND && !low_side)) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();

    switch (m) {
    case VIF_PIN_OFF:
        if (high_side) {
            if (g.hs_pin != obd_pin) {
                break; /* nobody is driving it; nothing to switch off */
            }
            if (g.hs_owner != owner) {
                UNLOCK();
                return ESP_ERR_INVALID_STATE;
            }
            hs_release_locked();
        } else {
            if (g.ls_pin != obd_pin) {
                break;
            }
            if (g.ls_owner != owner) {
                UNLOCK();
                return ESP_ERR_INVALID_STATE;
            }
            ls_release_locked();
        }
        break;

    case VIF_PIN_VOLTAGE:
        if (g.ls_pin == obd_pin) {
            UNLOCK();
            return ESP_ERR_INVALID_STATE; /* already pulled to ground */
        }
        if (g.hs_pin != -1 && (g.hs_pin != obd_pin || g.hs_owner != owner)) {
            /* SAE J2534 allows one high side driver at a time. */
            UNLOCK();
            return ESP_ERR_INVALID_STATE;
        }

        board_set_hs_voltage(mv);
        board_set_hs_boost_en(1);
        board_set_hs_state((enum hs_pin)obd_pin, 1);
        g.hs_pin = (int8_t)obd_pin;
        g.hs_owner = owner;
        break;

    case VIF_PIN_GROUND:
        if (g.hs_pin == obd_pin) {
            UNLOCK();
            return ESP_ERR_INVALID_STATE; /* already driven from the boost rail
                                           */
        }
        if (g.ls_pin != -1 && (g.ls_pin != obd_pin || g.ls_owner != owner)) {
            UNLOCK();
            return ESP_ERR_INVALID_STATE;
        }

        board_set_ls_state((enum ls_pin)obd_pin, 1);
        g.ls_pin = (int8_t)obd_pin;
        g.ls_owner = owner;
        break;
    }

    led_refresh_locked();
    UNLOCK();

    return ESP_OK;
}

esp_err_t vif_pin_release_all(vif_owner_t owner) {
    if (owner >= VIF_OWNER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();

    if (g.hs_pin != -1 && g.hs_owner == owner) {
        hs_release_locked();
    }
    if (g.ls_pin != -1 && g.ls_owner == owner) {
        ls_release_locked();
    }

    led_refresh_locked();
    UNLOCK();

    return ESP_OK;
}

bool vif_any_pin_active(void) {
    bool live;

    LOCK();
    live = (g.hs_pin != -1 || g.ls_pin != -1);
    UNLOCK();

    return live;
}

int32_t vif_vbatt_mv(void) { return board_get_vbatt(); }

int32_t vif_hs_vsense_mv(void) { return board_get_hs_vsense(); }

/**
 * @brief Take the pin claim for a factory procedure that drives them itself.
 *
 * board_calibrate_hs() starts and ends with board_hs_ls_reset_state(), which
 * switches every driver off. Anything this owner was already driving is
 * therefore released here first, or the bookkeeping would go on claiming a pin
 * the calibration had quietly switched off.
 */
static esp_err_t pins_claim_for_calibration(vif_owner_t owner) {
    LOCK();

    if ((g.hs_pin != -1 && g.hs_owner != owner) ||
        (g.ls_pin != -1 && g.ls_owner != owner)) {
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }

    hs_release_locked();
    ls_release_locked();

    g.hs_owner = owner;
    g.hs_pin = VIF_CAL_HS_PIN;
    led_refresh_locked();

    UNLOCK();
    return ESP_OK;
}

static void pins_drop_after_calibration(vif_owner_t owner) {
    LOCK();

    if (g.hs_pin != -1 && g.hs_owner == owner) {
        /* board_calibrate_hs() puts the drivers back itself, so this is only
         * the bookkeeping. */
        g.hs_pin = -1;
    }

    led_refresh_locked();
    UNLOCK();
}

esp_err_t vif_calibrate_hs(vif_owner_t owner) {
    esp_err_t err;

    if (owner >= VIF_OWNER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    err = pins_claim_for_calibration(owner);
    if (err != ESP_OK) {
        return err;
    }

    /* Prompts on the console and waits for a person, so the claim lock is not
     * held across it. */
    board_calibrate_hs();

    pins_drop_after_calibration(owner);
    return ESP_OK;
}

esp_err_t vif_calibrate_vbatt(void) {
    /* Reads the battery input only; no pin is driven, so no claim is needed. */
    board_calibrate_vbatt();
    return ESP_OK;
}

esp_err_t vif_calibration_set(const char *name, int32_t value) {
    if (!name) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The constants turn a requested millivolt into a duty cycle, so moving
     * them under a pin that is already energised would change what the next
     * write to that pin does. Wait until the connector is quiet instead. */
    if (vif_any_pin_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    return board_calibration_set(name, value);
}

/* ------------------------------------------------------------------ *
 * Init and diagnostics
 * ------------------------------------------------------------------ */

esp_err_t vif_init(void) {
    if (!g.lock) {
        g.lock = xSemaphoreCreateMutex();
        if (!g.lock) {
            ESP_LOGE(TAG, "failed to create the claim lock");
            return ESP_ERR_NO_MEM;
        }
    }

    LOCK();
    memset(g.task, 0, sizeof(g.task));
    memset((void *)g.recover_req, 0, sizeof(g.recover_req));
    memset((void *)g.release_req, 0, sizeof(g.release_req));
    memset(g.claim, 0, sizeof(g.claim));
    memset(&g.link, 0, sizeof(g.link));
    g.link.port = COMM_INVALID_PORT_ID;
    g.hs_pin = -1;
    g.ls_pin = -1;
    g.inited = true;
    UNLOCK();

    /* vif owns the board, so bringing it up is part of coming up. */
    board_setup();
    board_hs_ls_reset_state();

    return ESP_OK;
}

/** @brief Every bus @p owner holds, comma separated, into @p dst. */
static void owner_buses_locked(vif_owner_t owner, char *dst, size_t cap) {
    size_t pos = 0;

    dst[0] = '\0';

    for (int i = 0; i < VIF_RES_COUNT; i++) {
        int n;

        if (g.claim[i].bus == VIF_BUS_NONE || g.claim[i].owner != owner) {
            continue;
        }

        n = snprintf(dst + pos, cap - pos, "%s%s", pos ? "," : "",
                     vif_bus_name(g.claim[i].bus));
        if (n < 0 || (size_t)n >= cap - pos) {
            break;
        }
        pos += (size_t)n;
    }

    if (!pos) {
        snprintf(dst, cap, "-");
    }
}

void vif_print_debug_info(void) {
    struct {
        char buses[32];
        int hs;
        int ls;
    } snap[VIF_OWNER_COUNT];
    vif_bus_claim_t claim[VIF_BUS_GROUPS];
    const char *fe;
    comm_port_id_t port;
    char port_txt[4];

    /* Snapshot under the lock, print outside it: the console is slow enough
     * that holding the lock across it would stall the link. */
    LOCK();
    for (int i = 0; i < VIF_RES_COUNT; i++) {
        claim[i].bus = g.claim[i].bus;
        claim[i].bitrate = g.claim[i].bitrate;
        claim[i].owner = g.claim[i].owner;
    }
    for (int i = 0; i < VIF_OWNER_COUNT; i++) {
        vif_owner_t owner = (vif_owner_t)i;

        snap[i].hs = (g.hs_pin != -1 && g.hs_owner == owner) ? g.hs_pin : -1;
        snap[i].ls = (g.ls_pin != -1 && g.ls_owner == owner) ? g.ls_pin : -1;
        owner_buses_locked(owner, snap[i].buses, sizeof(snap[i].buses));
    }
    fe = g.link.fe ? g.link.fe->name : "-";
    port = g.link.port;
    UNLOCK();

    if (port == COMM_INVALID_PORT_ID) {
        snprintf(port_txt, sizeof(port_txt), "-");
    } else {
        snprintf(port_txt, sizeof(port_txt), "%u", (unsigned)port);
    }

    printf("Vehicle interface\n");
    printf("  link front-end %s on port %s\n", fe, port_txt);

    for (int i = 0; i < VIF_BUS_GROUPS; i++) {
        if (claim[i].bus == VIF_BUS_NONE) {
            continue;
        }

        printf("  %s", vif_bus_name(claim[i].bus));
        if (claim[i].bitrate) {
            printf(" @ %u", (unsigned)claim[i].bitrate);
        }
        printf(", held by %s\n", vif_owner_name(claim[i].owner));
    }

    printf("  %-8s %-20s %-4s %-4s\n", "owner", "buses", "HS", "LS");

    for (int i = 0; i < VIF_OWNER_COUNT; i++) {
        printf("  %-8s %-20s %-4d %-4d\n", vif_owner_name((vif_owner_t)i),
               snap[i].buses, snap[i].hs, snap[i].ls);
    }
}
