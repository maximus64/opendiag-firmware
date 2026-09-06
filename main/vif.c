/* SPDX-License-Identifier: GPL-3.0-only */
#include "vif.h"
#include <ctype.h>
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

/** Idle wait in the session task. Also how long a front-end switch requested
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

/** How long vif_session_close() waits for a running session task to finish. */
#define VIF_STOP_WAIT_MS 5000

/** Pin the high side calibration procedure drives. */
#define VIF_CAL_HS_PIN 12

struct vif_session {
    bool in_use;
    char name[VIF_NAME_LEN];
    vif_session_kind_t kind;

    /* The transport, which a link's client brings with it and takes away
     * again. Read on every pass of the session task and by every front-end,
     * written under the lock. */
    volatile comm_port_id_t port;

    /* Front-end. fe/fe_ctx are only ever written by the session's own task
     * once that task exists, so feed() cannot be running against a context
     * that destroy() has already freed. */
    const vif_frontend_t *fe;
    void *fe_ctx;
    const vif_frontend_t *fe_next;
    bool fe_switch; /* set under the lock, consumed by the session task */

    /* What the link goes back to when its client detaches. */
    const vif_frontend_t *fe_default;

    TaskHandle_t task;
    TaskHandle_t creator_task;
    volatile bool stop;
    volatile bool exited;

    /* Both consumed by vif_session_service() on the thread that owns this
     * session: recover_req from the front panel button, release_req from
     * anybody who asked for the claims to go while standing on the wrong
     * thread to take a driver down. */
    volatile bool recover_req;
    volatile bool release_req;
};

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

    vif_session_t sessions[VIF_MAX_SESSIONS];

    /* One live bus per group, and who opened it. */
    struct {
        vif_bus_t bus;
        bool open;
        uint32_t bitrate;
        vif_session_t *owner;
    } claim[VIF_RES_COUNT];

    /* SAE J2534: one high side and one low side driver at a time. -1 when the
     * driver is idle; the owner says which session may switch it off. */
    int8_t hs_pin;
    int8_t ls_pin;
    vif_session_t *hs_owner;
    vif_session_t *ls_owner;
} g;

#define LOCK() xSemaphoreTake(g.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g.lock)

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
 * @brief Is this the thread that could be inside @p s 's drivers?
 *
 * A transfer checks the claim under the lock and then lets go of it, because
 * a receive parks for as long as its timeout and holding the lock that long
 * would stall every other session. So while a transfer is in flight, nothing
 * says so - and a close arriving from another thread would free the queue,
 * the channels and the transmit mutex under a task still using them. On a
 * long block transfer that window is seconds wide, and the consequence is a
 * crash in the middle of a programming session.
 *
 * The rule that closes it: a driver is torn down only by the thread that
 * could be blocked in it. A session with a task of its own is serviced by
 * that task and nothing else; one without - the shell - is driven by its
 * creating task. Anybody else asks, and vif_session_service() does it.
 */
static bool on_owner_thread(const vif_session_t *s) {
    TaskHandle_t owner = s->task ? s->task : s->creator_task;
    return owner == xTaskGetCurrentTaskHandle();
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
    g.hs_owner = NULL;
}

static void ls_release_locked(void) {
    if (g.ls_pin == -1) {
        return;
    }

    board_set_ls_state((enum ls_pin)g.ls_pin, 0);

    g.ls_pin = -1;
    g.ls_owner = NULL;
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
 * Sessions
 * ------------------------------------------------------------------ */

/** @brief Bring @p fe up on @p s. False when its create() refused. */
static bool frontend_start(vif_session_t *s, const vif_frontend_t *fe) {
    void *ctx = NULL;

    if (fe->create) {
        ctx = fe->create(s);
        if (!ctx) {
            return false;
        }
    }

    s->fe = fe;
    s->fe_ctx = ctx;

    /* Whatever the previous front-end still had to say has been said and
     * dropped by now - its destroy() ran before this - so the port is this
     * one's to answer on. Harmless when nothing muted it. */
    comm_port_client_ready(s->port);

    ESP_LOGI(TAG, "%s: front-end %s", s->name, fe->name);
    return true;
}

/** Install @p fe on @p s. Old frontend torn down first; previous restored on
 *  refusal so a bad switch doesn't leave the port mute. Releases every claim.
 */
static bool session_apply_frontend(vif_session_t *s, const vif_frontend_t *fe) {
    const vif_frontend_t *prev = s->fe;

    if (prev && prev->destroy) {
        prev->destroy(s->fe_ctx);
    }
    s->fe = NULL;
    s->fe_ctx = NULL;

    /* A grammar change is a fresh start. The bus the last one left open was
     * claimed for its protocol, and the timings and filters on it mean
     * nothing to the next; destroy() has already had its chance to close the
     * session down politely. */
    vif_pin_release_all(s);
    if (vif_bus_release_all(s) != ESP_OK) {
        return false;
    }

    if (!fe || frontend_start(s, fe)) {
        return true;
    }

    ESP_LOGE(TAG, "%s: front-end %s refused to start", s->name, fe->name);

    if (prev && !frontend_start(s, prev)) {
        ESP_LOGE(TAG, "%s: %s did not come back either", s->name, prev->name);
    }
    return true;
}

static void session_task(void *param) {
    vif_session_t *s = param;
    uint8_t buf[64];

    while (!s->stop) {
        const vif_frontend_t *fe = NULL;
        const vif_frontend_t *next = NULL;
        bool switching = false;
        comm_port_id_t port;
        TickType_t wait;
        size_t n;

        /* Ahead of the switch below, so a default grammar this puts back is
         * installed in the same pass rather than the next one. */
        vif_session_service(s);

        /* Read the request under the lock, apply it outside: create() and
         * destroy() reach back into vif for their own claims. */
        LOCK();
        if (s->fe_switch) {
            s->fe_switch = false;
            next = s->fe_next;
            switching = true;
        }
        UNLOCK();

        if (switching) {
            if (!session_apply_frontend(s, next)) {
                LOCK();
                if (!s->fe_switch) {
                    s->fe_next = next;
                    s->fe_switch = true;
                }
                UNLOCK();
                vTaskDelay(wait_ticks(VIF_IDLE_WAIT_MS));
                continue;
            }
        }

        fe = s->fe;
        port = s->port;

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
            fe->feed(s->fe_ctx, buf, n);
        }
        if (fe->poll) {
            fe->poll(s->fe_ctx);
        }
    }

    while (!session_apply_frontend(s, NULL)) {
        vTaskDelay(wait_ticks(VIF_IDLE_WAIT_MS));
    }

    ESP_LOGI(TAG, "%s: session task exiting", s->name);
    s->exited = true;
    vTaskDelete(NULL);
}

vif_session_t *vif_session_open(const char *name, vif_session_kind_t kind) {
    vif_session_t *s = NULL;

    if (!g.inited) {
        ESP_LOGE(TAG, "vif_init() has not run");
        return NULL;
    }

    LOCK();

    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        if (!g.sessions[i].in_use) {
            s = &g.sessions[i];
            break;
        }
    }

    if (!s) {
        UNLOCK();
        ESP_LOGE(TAG, "no free session for '%s'", name ? name : "?");
        return NULL;
    }

    memset(s, 0, sizeof(*s));
    s->in_use = true;
    s->kind = kind;
    s->creator_task = xTaskGetCurrentTaskHandle();
    s->port = COMM_INVALID_PORT_ID;
    snprintf(s->name, sizeof(s->name), "%s", name ? name : "?");

    UNLOCK();

    ESP_LOGI(TAG, "session '%s' opened", s->name);
    return s;
}

void vif_session_close(vif_session_t *s) {
    if (!s || !s->in_use) {
        return;
    }

    if (s->task) {
        /* Ask the task to leave rather than deleting it where it stands: it
         * could be holding the claim lock, or halfway through a bus transfer.
         */
        s->stop = true;

        for (int waited = 0; !s->exited && waited < VIF_STOP_WAIT_MS;
             waited += VIF_IDLE_WAIT_MS) {
            vTaskDelay(wait_ticks(VIF_IDLE_WAIT_MS));
        }

        if (!s->exited) {
            /* Freeing the slot now would hand a live task's session to the
             * next caller. Keeping it is the lesser leak. */
            ESP_LOGE(TAG, "%s: session task did not exit; slot kept", s->name);
            return;
        }
    } else {
        if (!on_owner_thread(s)) {
            ESP_LOGE(TAG, "%s: session closed from the wrong thread", s->name);
            return;
        }
        if (!session_apply_frontend(s, NULL)) {
            ESP_LOGE(TAG, "%s: bus did not close; slot kept", s->name);
            return;
        }
    }

    LOCK();
    memset(s, 0, sizeof(*s));
    UNLOCK();
}

esp_err_t vif_session_set_frontend(vif_session_t *s, const vif_frontend_t *fe) {
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s->task) {
        /* Hand the swap to the session task, so destroy() and create() cannot
         * run underneath a feed() that is mid-command. */
        LOCK();
        s->fe_next = fe;
        s->fe_switch = true;
        UNLOCK();
        return ESP_OK;
    }

    if (!on_owner_thread(s)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!session_apply_frontend(s, fe)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (fe && s->fe != fe) {
        return ESP_FAIL;
    }

    /* A link runs its grammar whether or not a client is attached: the task
     * is what applies a later switch, and the shell sets a link's mode before
     * the tool that needs it connects. */
    if (s->fe && s->kind == VIF_SESSION_LINK &&
        xTaskCreate(session_task, s->name, 4 * 1024, s, 5, &s->task) !=
            pdPASS) {
        ESP_LOGE(TAG, "%s: failed to start session task", s->name);
        return ESP_FAIL;
    }

    return ESP_OK;
}

comm_port_id_t vif_session_port(const vif_session_t *s) {
    if (!s || !s->in_use) {
        return COMM_INVALID_PORT_ID;
    }

    return s->port;
}

esp_err_t vif_session_set_port(vif_session_t *s, comm_port_id_t port) {
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();
    s->port = port;
    UNLOCK();

    if (port == COMM_INVALID_PORT_ID) {
        ESP_LOGI(TAG, "%s: transport detached", s->name);
        return ESP_OK;
    }

    /* Muted when its last client left; this one may be answered. */
    comm_port_client_ready(port);

    ESP_LOGI(TAG, "%s: on port %u", s->name, (unsigned)port);
    return ESP_OK;
}

const char *vif_session_name(const vif_session_t *s) {
    if (!s || !s->in_use) {
        return NULL;
    }
    return s->name;
}

bool vif_session_is_link(const vif_session_t *s) {
    return s && s->in_use && s->kind == VIF_SESSION_LINK;
}

const char *vif_session_frontend_name(const vif_session_t *s) {
    if (!s || !s->in_use || !s->fe) {
        return NULL;
    }
    return s->fe->name;
}

esp_err_t vif_session_set_default_frontend(vif_session_t *s,
                                           const vif_frontend_t *fe) {
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();
    s->fe_default = fe;
    UNLOCK();

    return ESP_OK;
}

const vif_frontend_t *vif_session_default_frontend(const vif_session_t *s) {
    if (!s || !s->in_use) {
        return NULL;
    }
    return s->fe_default;
}

void vif_session_link_down(vif_session_t *s) {
    const vif_frontend_t *want;

    if (!s || !s->in_use) {
        return;
    }

    /* Flush stale input and mute the port before the next client arrives. */
    comm_port_client_gone(s->port);

    LOCK();
    want = s->fe_default;
    UNLOCK();

    if (!want) {
        /* No default to restart: lift the mute and keep the current frontend.
         */
        comm_port_client_ready(s->port);
        return;
    }

    /* Reinstall unconditionally — the departed client's parser state (echo,
     * headers, line buffer) must not reach the next client. */
    ESP_LOGI(TAG, "%s: client gone, restarting %s", s->name, want->name);

    vif_session_set_frontend(s, want);

    /* A disconnected session holds the hardware against everybody else, so
     * it lets go here rather than at the next reconnect. Pins go at once -
     * nothing blocks on a GPIO - but the buses go through the request, since
     * this runs on the transport's thread and the session's own may be
     * parked inside a driver. */
    vif_pin_release_all(s);
    vif_bus_release_all(s);
}

vif_session_t *vif_session_at(size_t idx) {
    vif_session_t *s;

    if (idx >= VIF_MAX_SESSIONS) {
        return NULL;
    }

    LOCK();
    s = g.sessions[idx].in_use ? &g.sessions[idx] : NULL;
    UNLOCK();

    return s;
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
        if (g.claim[i].owner) {
            snprintf(out[i].owner, sizeof(out[i].owner), "%s",
                     g.claim[i].owner->name);
        }
    }
    UNLOCK();
}

vif_session_t *vif_session_find(const char *name) {
    vif_session_t *found = NULL;

    if (!name) {
        return NULL;
    }

    LOCK();
    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        if (g.sessions[i].in_use && strcmp(g.sessions[i].name, name) == 0) {
            found = &g.sessions[i];
            break;
        }
    }
    UNLOCK();

    return found;
}

void vif_recover_all(void) {
    /* No logging: the front panel button's watcher calls this, and it is sized
     * on the promise that nothing here reaches vprintf. Each session announces
     * itself from vif_session_service() instead. */
    LOCK();
    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        if (g.sessions[i].in_use) {
            g.sessions[i].recover_req = true;
        }
    }
    UNLOCK();
}

bool vif_session_service(vif_session_t *s) {
    bool recover;
    bool release;

    if (!s || !s->in_use || !on_owner_thread(s)) {
        return false;
    }

    LOCK();
    recover = s->recover_req;
    release = s->release_req;
    s->recover_req = false;
    s->release_req = false;
    UNLOCK();

    if (!recover && !release) {
        return false;
    }

    if (recover) {
        ESP_LOGW(TAG, "%s: releasing every claim", s->name);
    }

    /* Safe here and only here: this thread owns the session, so it is not
     * sitting inside a driver call on the bus about to be torn down. */
    esp_err_t err = vif_bus_release_all(s);
    vif_pin_release_all(s);
    if (err != ESP_OK) {
        LOCK();
        s->release_req = true;
        s->recover_req |= recover;
        UNLOCK();
        return true;
    }

    /* A recovery puts the grammar back too. A release does not: whoever
     * asked for it has already said what should run next. */
    if (recover && s->fe_default) {
        vif_session_set_frontend(s, s->fe_default);
    }

    return true;
}

/* ------------------------------------------------------------------ *
 * Buses
 * ------------------------------------------------------------------ */

esp_err_t vif_bus_open(vif_session_t *s, vif_bus_t bus,
                       const vif_bus_cfg_t *cfg) {
    vif_res_t grp = bus_group(bus);
    esp_err_t err;

    if (!s || !s->in_use || grp == VIF_RES_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!on_owner_thread(s)) {
        return ESP_ERR_INVALID_STATE;
    }

    LOCK();

    if (g.claim[grp].bus != VIF_BUS_NONE && g.claim[grp].owner != s) {
        vif_bus_t held = g.claim[grp].bus;
        char owner[sizeof(s->name)];
        snprintf(owner, sizeof(owner), "%s",
                 g.claim[grp].owner ? g.claim[grp].owner->name : "?");
        UNLOCK();
        ESP_LOGW(TAG, "%s: %s is held by %s", s->name, vif_bus_name(held),
                 owner);
        return ESP_ERR_INVALID_STATE;
    }

    vif_bus_t old = g.claim[grp].bus;
    g.claim[grp].open = false;
    if (old == VIF_BUS_NONE) {
        g.claim[grp].bus = bus;
        g.claim[grp].owner = s;
    }
    UNLOCK();

    /* The reservation excludes other owners while the driver blocks. */
    if (old != VIF_BUS_NONE) {
        err = bus_driver_down(old);
        if (err != ESP_OK)
            return err;
    }

    LOCK();
    g.claim[grp].bus = bus;
    g.claim[grp].bitrate = cfg ? cfg->bitrate : 0;
    g.claim[grp].owner = s;
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
        ESP_LOGE(TAG, "%s: %s failed to start: %s", s->name, vif_bus_name(bus),
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "%s: %s bus open", s->name, vif_bus_name(bus));
    return ESP_OK;
}

esp_err_t vif_bus_close(vif_session_t *s, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);

    if (!s || !s->in_use || grp == VIF_RES_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!on_owner_thread(s)) {
        /* Every caller of this closes a bus it holds itself, so reaching
         * here is a mistake rather than a case to handle: releasing on
         * somebody's behalf is vif_bus_release_all()'s job. */
        ESP_LOGE(TAG, "%s: %s closed from the wrong thread", s->name,
                 vif_bus_name(bus));
        return ESP_ERR_INVALID_STATE;
    }

    LOCK();

    if (g.claim[grp].bus == VIF_BUS_NONE) {
        UNLOCK();
        return ESP_OK;
    }
    if (g.claim[grp].owner != s) {
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

esp_err_t vif_bus_release_all(vif_session_t *s) {
    esp_err_t result = ESP_OK;
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!on_owner_thread(s)) {
        /* Prompt rather than instant: the owner acts when its receive times
         * out. That is the price of never freeing a driver underneath it. */
        LOCK();
        s->release_req = true;
        UNLOCK();
        return ESP_OK;
    }

    for (int i = 0; i < VIF_RES_COUNT; i++) {
        LOCK();
        if (g.claim[i].owner != s) {
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

vif_bus_t vif_bus_current(const vif_session_t *s, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    vif_bus_t held;

    if (!s || !s->in_use || grp == VIF_RES_COUNT) {
        return VIF_BUS_NONE;
    }

    LOCK();
    held = (g.claim[grp].owner == s) ? g.claim[grp].bus : VIF_BUS_NONE;
    UNLOCK();

    return held;
}

bool vif_bus_is_open(const vif_session_t *s, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    if (!s || !s->in_use || grp == VIF_RES_COUNT)
        return false;
    LOCK();
    bool open =
        g.claim[grp].owner == s && g.claim[grp].bus == bus && g.claim[grp].open;
    UNLOCK();
    return open;
}

/**
 * @brief Confirm @p s holds @p bus itself, not merely its group.
 *
 * The transfer calls check the claim under the lock and then let go of it: a
 * receive parks for as long as its timeout, and holding the claim lock that
 * long would stall every other session's commands. Only the owning session
 * can close a bus, and a session is driven by one task, so the claim cannot
 * evaporate underneath its own transfer.
 */
static bool bus_owned_by(vif_session_t *s, vif_bus_t bus) {
    vif_res_t grp = bus_group(bus);
    bool ok;

    if (!s || !s->in_use || grp == VIF_RES_COUNT || !on_owner_thread(s)) {
        return false;
    }

    LOCK();
    ok = (g.claim[grp].bus == bus && g.claim[grp].owner == s &&
          g.claim[grp].open);
    UNLOCK();

    return ok;
}

int vif_bus_send(vif_session_t *s, vif_bus_t bus, const bus_msg_t *msg,
                 uint32_t flags) {
    const bus_ops_t *ops = bus_for(bus);

    if (!msg || !msg->data || !ops || !bus_owned_by(s, bus)) {
        return VIF_ERR_NO_CLAIM;
    }

    /* What counts as an empty message is the bus's own business: nothing to
     * say on a byte bus, but a legal frame on CAN, where the length code can
     * be zero. Each driver refuses its own. */
    return ops->send(msg, flags);
}

int vif_bus_recv(vif_session_t *s, vif_bus_t bus, bus_msg_t *msg,
                 TickType_t wait) {
    const bus_ops_t *ops = bus_for(bus);

    if (!msg || !msg->data || !ops || !bus_owned_by(s, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (msg->cap == 0) {
        return BUS_ERR_BAD_ARG;
    }

    return ops->recv(msg, wait);
}

int vif_bus_param_set(vif_session_t *s, vif_bus_t bus, bus_param_t p,
                      uint32_t value) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !bus_owned_by(s, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (!ops->set_param) {
        return BUS_ERR_UNSUPPORTED;
    }

    return ops->set_param(p, value);
}

int vif_bus_param_get(vif_session_t *s, vif_bus_t bus, bus_param_t p,
                      uint32_t *out) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !out || !bus_owned_by(s, bus)) {
        return VIF_ERR_NO_CLAIM;
    }
    if (!ops->get_param) {
        return BUS_ERR_UNSUPPORTED;
    }

    return ops->get_param(p, out);
}

int vif_bus_ioctl(vif_session_t *s, vif_bus_t bus, bus_ioctl_t id,
                  const void *in, void *out) {
    const bus_ops_t *ops = bus_for(bus);

    if (!ops || !bus_owned_by(s, bus)) {
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

esp_err_t vif_pin_set(vif_session_t *s, int obd_pin, vif_pin_mode_t m,
                      uint32_t mv) {
    bool high_side = (obd_pin == HS_OBD_PIN_6 || obd_pin == HS_OBD_PIN_9 ||
                      obd_pin == HS_OBD_PIN_11 || obd_pin == HS_OBD_PIN_12 ||
                      obd_pin == HS_OBD_PIN_13 || obd_pin == HS_OBD_PIN_14);
    bool low_side = (obd_pin == LS_OBD_PIN_15);

    if (!s || !s->in_use) {
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
            if (g.hs_owner != s) {
                UNLOCK();
                return ESP_ERR_INVALID_STATE;
            }
            hs_release_locked();
        } else {
            if (g.ls_pin != obd_pin) {
                break;
            }
            if (g.ls_owner != s) {
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
        if (g.hs_pin != -1 && (g.hs_pin != obd_pin || g.hs_owner != s)) {
            /* SAE J2534 allows one high side driver at a time. */
            UNLOCK();
            return ESP_ERR_INVALID_STATE;
        }

        board_set_hs_voltage(mv);
        board_set_hs_boost_en(1);
        board_set_hs_state((enum hs_pin)obd_pin, 1);
        g.hs_pin = (int8_t)obd_pin;
        g.hs_owner = s;
        break;

    case VIF_PIN_GROUND:
        if (g.hs_pin == obd_pin) {
            UNLOCK();
            return ESP_ERR_INVALID_STATE; /* already driven from the boost rail
                                           */
        }
        if (g.ls_pin != -1 && (g.ls_pin != obd_pin || g.ls_owner != s)) {
            UNLOCK();
            return ESP_ERR_INVALID_STATE;
        }

        board_set_ls_state((enum ls_pin)obd_pin, 1);
        g.ls_pin = (int8_t)obd_pin;
        g.ls_owner = s;
        break;
    }

    led_refresh_locked();
    UNLOCK();

    return ESP_OK;
}

esp_err_t vif_pin_release_all(vif_session_t *s) {
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();

    if (g.hs_owner == s) {
        hs_release_locked();
    }
    if (g.ls_owner == s) {
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
 * switches every driver off. Anything this session was already driving is
 * therefore released here first, or the bookkeeping would go on claiming a pin
 * the calibration had quietly switched off.
 */
static esp_err_t pins_claim_for_calibration(vif_session_t *s) {
    LOCK();

    if ((g.hs_owner && g.hs_owner != s) || (g.ls_owner && g.ls_owner != s)) {
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }

    hs_release_locked();
    ls_release_locked();

    g.hs_owner = s;
    g.hs_pin = VIF_CAL_HS_PIN;
    led_refresh_locked();

    UNLOCK();
    return ESP_OK;
}

static void pins_drop_after_calibration(vif_session_t *s) {
    LOCK();

    if (g.hs_owner == s) {
        /* board_calibrate_hs() puts the drivers back itself, so this is only
         * the bookkeeping. */
        g.hs_pin = -1;
        g.hs_owner = NULL;
    }

    led_refresh_locked();
    UNLOCK();
}

esp_err_t vif_calibrate_hs(vif_session_t *s) {
    esp_err_t err;

    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    err = pins_claim_for_calibration(s);
    if (err != ESP_OK) {
        return err;
    }

    /* Prompts on the console and waits for a person, so the claim lock is not
     * held across it. */
    board_calibrate_hs();

    pins_drop_after_calibration(s);
    return ESP_OK;
}

esp_err_t vif_calibrate_vbatt(vif_session_t *s) {
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reads the battery input only; no pin is driven, so no claim is needed. */
    board_calibrate_vbatt();
    return ESP_OK;
}

esp_err_t vif_calibration_set(vif_session_t *s, const char *name,
                              int32_t value) {
    if (!s || !s->in_use || !name) {
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
    memset(g.sessions, 0, sizeof(g.sessions));
    memset(g.claim, 0, sizeof(g.claim));
    g.hs_pin = -1;
    g.ls_pin = -1;
    g.hs_owner = NULL;
    g.ls_owner = NULL;
    g.inited = true;
    UNLOCK();

    /* vif owns the board, so bringing it up is part of coming up. */
    board_setup();
    board_hs_ls_reset_state();

    return ESP_OK;
}

/** @brief Every bus @p s holds, comma separated, into @p dst. */
static void session_buses_locked(const vif_session_t *s, char *dst,
                                 size_t cap) {
    size_t pos = 0;

    dst[0] = '\0';

    for (int i = 0; i < VIF_RES_COUNT; i++) {
        int n;

        if (g.claim[i].owner != s) {
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
        char name[VIF_NAME_LEN];
        const char *fe;
        comm_port_id_t port;
        char buses[32];
        int hs;
        int ls;
    } snap[VIF_MAX_SESSIONS];
    vif_bus_claim_t claim[VIF_BUS_GROUPS];
    bool can_live = false;
    int count = 0;

    /* Snapshot under the lock, print outside it: the console is slow enough
     * that holding the lock across it would stall the sessions. */
    LOCK();
    for (int i = 0; i < VIF_RES_COUNT; i++) {
        claim[i].bus = g.claim[i].bus;
        claim[i].bitrate = g.claim[i].bitrate;
        claim[i].owner[0] = '\0';
        if (g.claim[i].owner) {
            snprintf(claim[i].owner, sizeof(claim[i].owner), "%s",
                     g.claim[i].owner->name);
        }
    }
    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        vif_session_t *s = &g.sessions[i];

        if (!s->in_use) {
            continue;
        }

        memcpy(snap[count].name, s->name, sizeof(snap[count].name));
        snap[count].fe = s->fe ? s->fe->name : "-";
        snap[count].port = s->port;
        snap[count].hs = (g.hs_owner == s) ? g.hs_pin : -1;
        snap[count].ls = (g.ls_owner == s) ? g.ls_pin : -1;
        session_buses_locked(s, snap[count].buses, sizeof(snap[count].buses));
        count++;
    }
    UNLOCK();

    printf("Vehicle interface\n");

    for (int i = 0; i < VIF_BUS_GROUPS; i++) {
        if (claim[i].bus == VIF_BUS_NONE) {
            continue;
        }

        printf("  %s", vif_bus_name(claim[i].bus));
        if (claim[i].bitrate) {
            printf(" @ %u", (unsigned)claim[i].bitrate);
        }
        printf(", held by %s\n", claim[i].owner[0] ? claim[i].owner : "?");

        can_live = can_live || claim[i].bus == VIF_BUS_CAN;
    }

    printf("  %-8s %-8s %-4s %-20s %-4s %-4s\n", "session", "frontend", "port",
           "buses", "HS", "LS");

    for (int i = 0; i < count; i++) {
        char port[4];

        if (snap[i].port == COMM_INVALID_PORT_ID) {
            snprintf(port, sizeof(port), "-");
        } else {
            snprintf(port, sizeof(port), "%u", (unsigned)snap[i].port);
        }

        printf("  %-8s %-8s %-4s %-20s %-4d %-4d\n", snap[i].name, snap[i].fe,
               port, snap[i].buses, snap[i].hs, snap[i].ls);
    }

    if (can_live) {
        can_print_stat();
    }
}
