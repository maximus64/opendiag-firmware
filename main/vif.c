/* SPDX-License-Identifier: GPL-3.0-only */
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "board.h"
#include "can_bus.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline.h"
#include "vif.h"
#include "ws2812_led.h"

#define TAG "vif"

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

/**
 * @brief Milliseconds to ticks, never rounding down to "do not wait".
 *
 * pdMS_TO_TICKS() truncates: at the ESP-IDF default of 100 Hz, anything under
 * 10 ms becomes zero ticks, which turns the session task's blocking read into
 * a poll and spins a core flat out. One tick is the floor.
 */
static inline TickType_t wait_ticks(uint32_t ms)
{
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
    comm_port_id_t port;

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
    volatile bool stop;
    volatile bool exited;

    /* Set by vif_recover_all(), consumed by vif_session_service() on the
     * thread that owns this session. */
    volatile bool recover_req;
};

static struct {
    SemaphoreHandle_t lock;
    bool inited;

    vif_session_t sessions[VIF_MAX_SESSIONS];

    /* The one live bus, and who opened it. */
    vif_bus_t bus;
    uint32_t bus_bitrate;
    vif_session_t *bus_owner;

    /* SAE J2534: one high side and one low side driver at a time. -1 when the
     * driver is idle; the owner says which session may switch it off. */
    int8_t hs_pin;
    int8_t ls_pin;
    vif_session_t *hs_owner;
    vif_session_t *ls_owner;
} g;

#define LOCK()   xSemaphoreTake(g.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g.lock)

/* ------------------------------------------------------------------ *
 * Drivers
 * ------------------------------------------------------------------ */

/**
 * @brief The three byte-oriented buses behind one shape.
 *
 * kline.c, j1850_pwm.c and j1850_vpw.c expose the same four calls, so the
 * dispatch is a table rather than a switch repeated in four places.
 */
struct bytebus_ops {
    void (*setup)(void);
    void (*teardown)(void);
    int (*send)(const uint8_t *data, uint8_t len);
    int (*recv)(uint8_t *data, uint8_t len, TickType_t wait);
};

static const struct bytebus_ops kline_ops = {
    kline_setup, kline_teardown, kline_send, kline_recieve,
};
static const struct bytebus_ops j1850_pwm_ops = {
    j1850_pwm_setup, j1850_pwm_teardown, j1850_pwm_send, j1850_pwm_receive,
};
static const struct bytebus_ops j1850_vpw_ops = {
    j1850_vpw_setup, j1850_vpw_teardown, j1850_vpw_send, j1850_vpw_receive,
};

static const struct bytebus_ops *bytebus_ops(vif_bus_t bus)
{
    switch (bus) {
    case VIF_BUS_KLINE:     return &kline_ops;
    case VIF_BUS_J1850_PWM: return &j1850_pwm_ops;
    case VIF_BUS_J1850_VPW: return &j1850_vpw_ops;
    default:                return NULL;
    }
}

const char *vif_bus_name(vif_bus_t bus)
{
    switch (bus) {
    case VIF_BUS_CAN:       return "CAN";
    case VIF_BUS_KLINE:     return "K-Line";
    case VIF_BUS_J1850_PWM: return "J1850 PWM";
    case VIF_BUS_J1850_VPW: return "J1850 VPW";
    default:                return "none";
    }
}

static esp_err_t bus_driver_up(vif_bus_t bus, const vif_bus_cfg_t *cfg)
{
    const struct bytebus_ops *ops;

    if (bus == VIF_BUS_CAN) {
        if (!cfg || cfg->bitrate == 0) {
            ESP_LOGE(TAG, "CAN needs a bit rate");
            return ESP_ERR_INVALID_ARG;
        }
        return can_bus_setup((int)cfg->bitrate);
    }

    ops = bytebus_ops(bus);
    if (!ops) {
        return ESP_ERR_INVALID_ARG;
    }

    ops->setup();

    if (bus == VIF_BUS_KLINE) {
        /* The 5 baud wakeup is part of bringing K-Line up for ISO 9141 and
         * KWP slow init, the only K-Line protocols implemented. When fast
         * init lands it becomes a field in vif_bus_cfg_t. */
        int rc = kline_sync();
        if (rc) {
            ESP_LOGW(TAG, "K-Line 5 baud init failed: %d", rc);
        }
    }

    return ESP_OK;
}

static void bus_driver_down(vif_bus_t bus)
{
    const struct bytebus_ops *ops;

    if (bus == VIF_BUS_CAN) {
        can_bus_teardown();
        return;
    }

    ops = bytebus_ops(bus);
    if (ops) {
        ops->teardown();
    }
}

/* ------------------------------------------------------------------ *
 * Pins
 * ------------------------------------------------------------------ */

static bool is_high_side_pin(int pin)
{
    switch (pin) {
    case HS_OBD_PIN_6:
    case HS_OBD_PIN_9:
    case HS_OBD_PIN_11:
    case HS_OBD_PIN_12:
    case HS_OBD_PIN_13:
    case HS_OBD_PIN_14:
        return true;
    default:
        return false;
    }
}

static bool is_low_side_pin(int pin)
{
    return pin == LS_OBD_PIN_15;
}

static bool pins_live(void)
{
    return g.hs_pin != -1 || g.ls_pin != -1;
}

/**
 * @brief Reflect the pin state on the status LED.
 *
 * A connector pin can be energised with no client attached - that is the whole
 * point of not tearing claims down on disconnect - so it has to be visible on
 * the device itself.
 */
static void led_refresh_locked(void)
{
    ws2812_led_set_state(pins_live() ? LED_STATE_PIN_LIVE : LED_STATE_IDLE);
}

static void hs_release_locked(void)
{
    if (g.hs_pin == -1) {
        return;
    }

    board_set_hs_state((enum hs_pin)g.hs_pin, 0);
    board_set_hs_boost_en(0);
    board_set_hs_voltage(0);

    g.hs_pin = -1;
    g.hs_owner = NULL;
}

static void ls_release_locked(void)
{
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

static int name_eq(const char *a, const char *b)
{
    if (!a || !b) {
        return 0;
    }

    for (; *a && *b; a++, b++) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return 0;
        }
    }

    return *a == *b;
}

esp_err_t vif_frontend_register(const vif_frontend_t *fe)
{
    if (!fe || !fe->name) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < g_frontend_count; i++) {
        if (g_frontends[i] == fe) {
            return ESP_OK;
        }
        if (name_eq(g_frontends[i]->name, fe->name)) {
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

const vif_frontend_t *vif_frontend_find(const char *name)
{
    for (size_t i = 0; i < g_frontend_count; i++) {
        if (name_eq(g_frontends[i]->name, name)) {
            return g_frontends[i];
        }
    }

    return NULL;
}

const vif_frontend_t *vif_frontend_at(size_t idx)
{
    return idx < g_frontend_count ? g_frontends[idx] : NULL;
}

/* ------------------------------------------------------------------ *
 * Sessions
 * ------------------------------------------------------------------ */

/** @brief Bring @p fe up on @p s. False when its create() refused. */
static bool frontend_start(vif_session_t *s, const vif_frontend_t *fe)
{
    void *ctx = NULL;

    if (fe->create) {
        ctx = fe->create(s, s->port);
        if (!ctx) {
            return false;
        }
    }

    s->fe = fe;
    s->fe_ctx = ctx;

    ESP_LOGI(TAG, "%s: front-end %s", s->name, fe->name);
    return true;
}

/**
 * @brief Install @p fe on @p s, tearing down whatever was there.
 *
 * Runs either in the session's task or, before that task exists, in the
 * caller's - never in both at once.
 *
 * The old front-end goes first, because a front-end that keeps its state in
 * file scope statics only runs one instance and will refuse to start while the
 * previous one is still up. If the new one refuses anyway, the previous is put
 * back: a mistyped switch should not leave the port mute until a power cycle.
 */
static void session_apply_frontend(vif_session_t *s, const vif_frontend_t *fe)
{
    const vif_frontend_t *prev = s->fe;

    if (prev && prev->destroy) {
        prev->destroy(s->fe_ctx);
    }
    s->fe = NULL;
    s->fe_ctx = NULL;

    if (!fe || frontend_start(s, fe)) {
        return;
    }

    ESP_LOGE(TAG, "%s: front-end %s refused to start", s->name, fe->name);

    if (prev && !frontend_start(s, prev)) {
        ESP_LOGE(TAG, "%s: %s did not come back either", s->name, prev->name);
    }
}

static void session_task(void *param)
{
    vif_session_t *s = param;
    uint8_t buf[64];

    while (!s->stop) {
        const vif_frontend_t *fe = NULL;
        const vif_frontend_t *next = NULL;
        bool switching = false;
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
            session_apply_frontend(s, next);
        }

        fe = s->fe;
        if (!fe) {
            vTaskDelay(wait_ticks(VIF_IDLE_WAIT_MS));
            continue;
        }

        /* A front-end with unsolicited output has to come round often enough
         * to drain the bus; one without can sleep until a byte shows up, bar
         * the wakeup that lets a front-end switch land. */
        wait = wait_ticks(fe->poll ? VIF_POLL_WAIT_MS : VIF_IDLE_WAIT_MS);

        n = comm_port_read(s->port, buf, sizeof(buf), wait);
        if (n && fe->feed) {
            fe->feed(s->fe_ctx, buf, n);
        }
        if (fe->poll) {
            fe->poll(s->fe_ctx);
        }
    }

    session_apply_frontend(s, NULL);

    ESP_LOGI(TAG, "%s: session task exiting", s->name);
    s->exited = true;
    vTaskDelete(NULL);
}

vif_session_t *vif_session_open(const char *name, comm_port_id_t port)
{
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
    s->port = port;
    snprintf(s->name, sizeof(s->name), "%s", name ? name : "?");

    UNLOCK();

    ESP_LOGI(TAG, "session '%s' opened on port %u", s->name, (unsigned)port);
    return s;
}

void vif_session_close(vif_session_t *s)
{
    if (!s || !s->in_use) {
        return;
    }

    if (s->task) {
        /* Ask the task to leave rather than deleting it where it stands: it
         * could be holding the claim lock, or halfway through a bus transfer. */
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
        s->task = NULL;
    }
    else {
        session_apply_frontend(s, NULL);
    }

    vif_bus_close(s);
    vif_pin_release_all(s);

    LOCK();
    memset(s, 0, sizeof(*s));
    UNLOCK();
}

esp_err_t vif_session_set_frontend(vif_session_t *s, const vif_frontend_t *fe)
{
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

    session_apply_frontend(s, fe);

    if (fe && s->fe != fe) {
        return ESP_FAIL;
    }

    if (s->fe && s->port != COMM_INVALID_PORT_ID) {
        if (xTaskCreate(session_task, s->name, 4 * 1024, s, 5, &s->task) != pdPASS) {
            ESP_LOGE(TAG, "%s: failed to start session task", s->name);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

const char *vif_session_name(const vif_session_t *s)
{
    if (!s || !s->in_use) {
        return NULL;
    }
    return s->name;
}

bool vif_session_is_link(const vif_session_t *s)
{
    return s && s->in_use && s->port != COMM_INVALID_PORT_ID;
}

const char *vif_session_frontend_name(const vif_session_t *s)
{
    if (!s || !s->in_use || !s->fe) {
        return NULL;
    }
    return s->fe->name;
}

esp_err_t vif_session_set_default_frontend(vif_session_t *s,
                                           const vif_frontend_t *fe)
{
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();
    s->fe_default = fe;
    UNLOCK();

    return ESP_OK;
}

const vif_frontend_t *vif_session_default_frontend(const vif_session_t *s)
{
    if (!s || !s->in_use) {
        return NULL;
    }
    return s->fe_default;
}

void vif_session_link_down(vif_session_t *s)
{
    const vif_frontend_t *want;
    bool revert;

    if (!s || !s->in_use) {
        return;
    }

    LOCK();
    want = s->fe_default;
    /* fe_next rather than fe when a switch is already queued: the queued one
     * is what the session will be running by the time this would take. */
    revert = want && (s->fe_switch ? s->fe_next : s->fe) != want;
    UNLOCK();

    if (!revert) {
        return;
    }

    ESP_LOGI(TAG, "%s: client gone, back to %s", s->name, want->name);

    /* Claims are untouched on purpose. Only the grammar is disposable. */
    vif_session_set_frontend(s, want);
}

vif_session_t *vif_session_at(size_t idx)
{
    vif_session_t *s;

    if (idx >= VIF_MAX_SESSIONS) {
        return NULL;
    }

    LOCK();
    s = g.sessions[idx].in_use ? &g.sessions[idx] : NULL;
    UNLOCK();

    return s;
}

void vif_bus_info(vif_bus_info_t *out)
{
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));

    LOCK();
    out->bus = g.bus;
    out->bitrate = g.bus_bitrate;
    if (g.bus_owner) {
        snprintf(out->owner, sizeof(out->owner), "%s", g.bus_owner->name);
    }
    UNLOCK();
}

vif_session_t *vif_session_find(const char *name)
{
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

void vif_recover_all(void)
{
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

bool vif_session_service(vif_session_t *s)
{
    bool wanted;

    if (!s || !s->in_use) {
        return false;
    }

    LOCK();
    wanted = s->recover_req;
    s->recover_req = false;
    UNLOCK();

    if (!wanted) {
        return false;
    }

    ESP_LOGW(TAG, "%s: releasing every claim", s->name);

    /* Safe here and only here: this thread owns the session, so it is not
     * sitting inside a driver call on the bus about to be torn down. */
    vif_bus_close(s);
    vif_pin_release_all(s);

    if (s->fe_default) {
        vif_session_set_frontend(s, s->fe_default);
    }

    return true;
}

/* ------------------------------------------------------------------ *
 * Buses
 * ------------------------------------------------------------------ */

esp_err_t vif_bus_open(vif_session_t *s, vif_bus_t bus, const vif_bus_cfg_t *cfg)
{
    esp_err_t err;

    if (!s || !s->in_use || bus == VIF_BUS_NONE) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();

    if (g.bus != VIF_BUS_NONE && g.bus_owner != s) {
        UNLOCK();
        ESP_LOGW(TAG, "%s: %s bus is held by %s", s->name, vif_bus_name(g.bus),
                 g.bus_owner ? g.bus_owner->name : "?");
        return ESP_ERR_INVALID_STATE;
    }

    /* Switching protocols within a session is one call: the old bus goes down
     * first, because two drivers cannot be live together. */
    if (g.bus != VIF_BUS_NONE) {
        bus_driver_down(g.bus);
        g.bus = VIF_BUS_NONE;
        g.bus_bitrate = 0;
        g.bus_owner = NULL;
    }

    err = bus_driver_up(bus, cfg);
    if (err != ESP_OK) {
        UNLOCK();
        ESP_LOGE(TAG, "%s: %s failed to start: %s", s->name, vif_bus_name(bus),
                 esp_err_to_name(err));
        return err;
    }

    g.bus = bus;
    g.bus_bitrate = (bus == VIF_BUS_CAN && cfg) ? cfg->bitrate : 0;
    g.bus_owner = s;

    UNLOCK();

    ESP_LOGI(TAG, "%s: %s bus open", s->name, vif_bus_name(bus));
    return ESP_OK;
}

esp_err_t vif_bus_close(vif_session_t *s)
{
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();

    if (g.bus == VIF_BUS_NONE) {
        UNLOCK();
        return ESP_OK;
    }
    if (g.bus_owner != s) {
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }

    bus_driver_down(g.bus);
    g.bus = VIF_BUS_NONE;
    g.bus_bitrate = 0;
    g.bus_owner = NULL;

    UNLOCK();
    return ESP_OK;
}

vif_bus_t vif_bus_current(const vif_session_t *s)
{
    vif_bus_t bus;

    if (!s || !s->in_use) {
        return VIF_BUS_NONE;
    }

    LOCK();
    bus = (g.bus_owner == s) ? g.bus : VIF_BUS_NONE;
    UNLOCK();

    return bus;
}

/**
 * @brief Confirm @p s may use @p bus right now.
 *
 * The transfer calls check the claim under the lock and then let go of it: a
 * receive parks for as long as its timeout, and holding the claim lock that
 * long would stall every other session's commands. Only the owning session
 * can close a bus, and a session is driven by one task, so the claim cannot
 * evaporate underneath its own transfer.
 */
static bool claim_ok(vif_session_t *s, vif_bus_t bus)
{
    bool ok;

    if (!s || !s->in_use) {
        return false;
    }

    LOCK();
    ok = (g.bus == bus && g.bus_owner == s);
    UNLOCK();

    return ok;
}

int vif_can_send(vif_session_t *s, const struct can_frame *f)
{
    if (!f || !claim_ok(s, VIF_BUS_CAN)) {
        return VIF_ERR_NO_CLAIM;
    }

    return can_send(f);
}

int vif_can_recv(vif_session_t *s, struct can_frame *f, TickType_t wait)
{
    if (!f || !claim_ok(s, VIF_BUS_CAN)) {
        return VIF_ERR_NO_CLAIM;
    }

    return can_receive(f, wait);
}

/** @brief The byte bus @p s holds, or NULL when it holds none. */
static const struct bytebus_ops *claimed_bytebus(vif_session_t *s)
{
    const struct bytebus_ops *ops = NULL;

    if (!s || !s->in_use) {
        return NULL;
    }

    LOCK();
    if (g.bus_owner == s) {
        ops = bytebus_ops(g.bus);
    }
    UNLOCK();

    return ops;
}

int vif_raw_send(vif_session_t *s, const uint8_t *d, size_t len)
{
    const struct bytebus_ops *ops = claimed_bytebus(s);

    if (!d || !ops) {
        return VIF_ERR_NO_CLAIM;
    }
    if (len == 0 || len > UINT8_MAX) {
        return -1;
    }

    return ops->send(d, (uint8_t)len);
}

int vif_raw_recv(vif_session_t *s, uint8_t *d, size_t cap, TickType_t wait)
{
    const struct bytebus_ops *ops = claimed_bytebus(s);

    if (!d || !ops) {
        return VIF_ERR_NO_CLAIM;
    }
    if (cap == 0) {
        return -1;
    }
    if (cap > UINT8_MAX) {
        cap = UINT8_MAX;
    }

    return ops->recv(d, (uint8_t)cap, wait);
}

/* ------------------------------------------------------------------ *
 * Pins and analog
 * ------------------------------------------------------------------ */

esp_err_t vif_pin_set(vif_session_t *s, int obd_pin, vif_pin_mode_t m, uint32_t mv)
{
    bool high_side = is_high_side_pin(obd_pin);
    bool low_side = is_low_side_pin(obd_pin);

    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!high_side && !low_side) {
        return ESP_ERR_INVALID_ARG;
    }
    /* The board drives voltage from the high side switches and ground from the
     * low side one; neither pin group can do the other's job. */
    if ((m == VIF_PIN_VOLTAGE && !high_side) || (m == VIF_PIN_GROUND && !low_side)) {
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
        }
        else {
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
            return ESP_ERR_INVALID_STATE; /* already driven from the boost rail */
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

esp_err_t vif_pin_release_all(vif_session_t *s)
{
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

bool vif_any_pin_active(void)
{
    bool live;

    LOCK();
    live = pins_live();
    UNLOCK();

    return live;
}

int32_t vif_vbatt_mv(void)
{
    return board_get_vbatt();
}

int32_t vif_hs_vsense_mv(void)
{
    return board_get_hs_vsense();
}

uint8_t vif_board_id(void)
{
    return board_get_boardid();
}

/**
 * @brief Take the pin claim for a factory procedure that drives them itself.
 *
 * board_calibrate_hs() starts and ends with board_hs_ls_reset_state(), which
 * switches every driver off. Anything this session was already driving is
 * therefore released here first, or the bookkeeping would go on claiming a pin
 * the calibration had quietly switched off.
 */
static esp_err_t pins_claim_for_calibration(vif_session_t *s)
{
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

static void pins_drop_after_calibration(vif_session_t *s)
{
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

esp_err_t vif_calibrate_hs(vif_session_t *s)
{
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

esp_err_t vif_calibrate_vbatt(vif_session_t *s)
{
    if (!s || !s->in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reads the battery input only; no pin is driven, so no claim is needed. */
    board_calibrate_vbatt();
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 * Init and diagnostics
 * ------------------------------------------------------------------ */

esp_err_t vif_init(void)
{
    if (!g.lock) {
        g.lock = xSemaphoreCreateMutex();
        if (!g.lock) {
            ESP_LOGE(TAG, "failed to create the claim lock");
            return ESP_ERR_NO_MEM;
        }
    }

    LOCK();
    memset(g.sessions, 0, sizeof(g.sessions));
    g.bus = VIF_BUS_NONE;
    g.bus_owner = NULL;
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

void vif_print_debug_info(void)
{
    struct {
        char name[VIF_NAME_LEN];
        const char *fe;
        comm_port_id_t port;
        bool bus;
        int hs;
        int ls;
    } snap[VIF_MAX_SESSIONS];
    vif_bus_t bus;
    int count = 0;

    /* Snapshot under the lock, print outside it: the console is slow enough
     * that holding the lock across it would stall the sessions. */
    LOCK();
    bus = g.bus;
    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        vif_session_t *s = &g.sessions[i];

        if (!s->in_use) {
            continue;
        }

        memcpy(snap[count].name, s->name, sizeof(snap[count].name));
        snap[count].fe = s->fe ? s->fe->name : "-";
        snap[count].port = s->port;
        snap[count].bus = (g.bus_owner == s);
        snap[count].hs = (g.hs_owner == s) ? g.hs_pin : -1;
        snap[count].ls = (g.ls_owner == s) ? g.ls_pin : -1;
        count++;
    }
    UNLOCK();

    printf("Vehicle interface\n");
    printf("  bus: %s\n", vif_bus_name(bus));
    printf("  %-8s %-8s %-4s %-10s %-4s %-4s\n",
           "session", "frontend", "port", "bus", "HS", "LS");

    for (int i = 0; i < count; i++) {
        char port[4];

        if (snap[i].port == COMM_INVALID_PORT_ID) {
            snprintf(port, sizeof(port), "-");
        }
        else {
            snprintf(port, sizeof(port), "%u", (unsigned)snap[i].port);
        }

        printf("  %-8s %-8s %-4s %-10s %-4d %-4d\n",
               snap[i].name, snap[i].fe, port,
               snap[i].bus ? vif_bus_name(bus) : "-", snap[i].hs, snap[i].ls);
    }

    if (bus == VIF_BUS_CAN) {
        can_print_stat();
    }
}
