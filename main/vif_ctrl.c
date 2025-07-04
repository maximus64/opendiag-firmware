/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file vif_ctrl.c
 * @brief The control plane's grammar. See vif_ctrl.h for what it is for.
 *
 * Two callers, one implementation: the shell's "mode" command and the BLE
 * control characteristic. Neither owns the command set, so the two cannot
 * drift apart.
 *
 * Nothing here touches hardware directly - it goes through the session like
 * every other client of vif, which is what keeps the claim rules in one place.
 */

#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "common.h"
#include "vif_ctrl.h"

#define TAG "vif_ctrl"

/** Longest command word we need to recognise, plus its argument. */
#define CTRL_CMD_MAX 48

static void reply(char *out, size_t cap, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void reply(char *out, size_t cap, const char *fmt, ...)
{
    va_list ap;

    if (!out || !cap) {
        return;
    }

    va_start(ap, fmt);
    vsnprintf(out, cap, fmt, ap);
    va_end(ap);
}

/**
 * @brief Trim, upper-case and NUL terminate one command into @p dst.
 *
 * A GATT write arrives as a whole message and the shell hands over a whole
 * line, so there is no partial-command state to keep. Trailing CR and LF are
 * tolerated because a terminal will send them and a phone app will not.
 *
 * @return false when the command is empty or too long to be one of ours.
 */
static bool normalise(const char *cmd, size_t len, char *dst, size_t cap)
{
    size_t n = 0;

    if (!cmd) {
        return false;
    }

    while (len && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n' ||
                   cmd[len - 1] == ' ' || cmd[len - 1] == '\0')) {
        len--;
    }
    while (len && *cmd == ' ') {
        cmd++;
        len--;
    }

    if (!len || len >= cap) {
        return false;
    }

    for (; n < len; n++) {
        dst[n] = (char)toupper((unsigned char)cmd[n]);
    }
    dst[n] = '\0';

    return true;
}

/** @brief Every registered grammar, comma separated, into @p dst. */
static void mode_list(char *dst, size_t cap)
{
    const vif_frontend_t *fe;
    size_t pos = 0;

    dst[0] = '\0';

    for (size_t i = 0; (fe = vif_frontend_at(i)) != NULL; i++) {
        int n = snprintf(dst + pos, cap - pos, "%s%s", pos ? "," : "", fe->name);

        if (n < 0 || (size_t)n >= cap - pos) {
            break;
        }
        pos += (size_t)n;
    }
}

static bool cmd_id(vif_session_t *s, char *out, size_t cap)
{
    char modes[64];
    const char *fe = vif_session_frontend_name(s);

    mode_list(modes, sizeof(modes));

    reply(out, cap, "%s %s fw=%s link=%s mode=%s modes=%s",
          OPENDIAG_PRODUCT, OPENDIAG_HARDWARE, OPENDIAG_VERSION,
          vif_session_name(s) ? vif_session_name(s) : "-",
          fe ? fe : "-", modes);

    return true;
}

static bool cmd_bus(char *out, size_t cap)
{
    vif_bus_info_t info;

    vif_bus_info(&info);

    if (info.bus == VIF_BUS_NONE) {
        reply(out, cap, "BUS none");
        return true;
    }

    if (info.bitrate) {
        reply(out, cap, "BUS %s %" PRIu32 " held=%s",
              vif_bus_name(info.bus), info.bitrate, info.owner);
    }
    else {
        reply(out, cap, "BUS %s held=%s", vif_bus_name(info.bus), info.owner);
    }

    return true;
}

static bool cmd_mode_set(vif_session_t *s, const char *name,
                         char *out, size_t cap)
{
    const vif_frontend_t *fe = vif_frontend_find(name);
    char modes[64];

    if (!fe) {
        mode_list(modes, sizeof(modes));
        reply(out, cap, "ERR no mode '%s'; have %s", name, modes);
        return false;
    }

    /*
     * The claims stay exactly where they are. That is the whole point of
     * switching inside the session rather than opening a second one: a
     * programming voltage raised under ELM327 is still up when the flash
     * starts under SLCAN.
     */
    if (vif_session_set_frontend(s, fe) != ESP_OK) {
        reply(out, cap, "ERR cannot switch to %s", fe->name);
        return false;
    }

    ESP_LOGI(TAG, "%s: mode %s", vif_session_name(s), fe->name);

    reply(out, cap, "MODE %s", fe->name);
    return true;
}

static bool cmd_reset(vif_session_t *s, char *out, size_t cap)
{
    const vif_frontend_t *fe = vif_session_default_frontend(s);

    /*
     * Unlike a client simply going away, this is an explicit request to let
     * go: the bus and any energised pin are released as well as the grammar.
     */
    vif_bus_close(s);
    vif_pin_release_all(s);

    if (fe) {
        vif_session_set_frontend(s, fe);
    }

    reply(out, cap, "RESET %s", fe ? fe->name : "-");
    return true;
}

bool vif_ctrl_exec(vif_session_t *s, const char *cmd, size_t len,
                   char *out, size_t cap)
{
    char buf[CTRL_CMD_MAX];

    if (!out || !cap) {
        return false;
    }

    if (!s) {
        reply(out, cap, "ERR no link");
        return false;
    }

    if (!normalise(cmd, len, buf, sizeof(buf))) {
        reply(out, cap, "ERR empty");
        return false;
    }

    if (strcmp(buf, "ID") == 0) {
        return cmd_id(s, out, cap);
    }

    if (strcmp(buf, "MODE") == 0 || strncmp(buf, "MODE=", 5) == 0) {
        /* Both forms are about a link's grammar, and the shell is not a link. */
        if (!vif_session_is_link(s)) {
            reply(out, cap, "ERR '%s' carries no protocol",
                  vif_session_name(s));
            return false;
        }

        if (buf[4] == '=') {
            return cmd_mode_set(s, &buf[5], out, cap);
        }

        reply(out, cap, "MODE %s", vif_session_frontend_name(s));
        return true;
    }

    if (strcmp(buf, "BUS") == 0) {
        return cmd_bus(out, cap);
    }

    if (strcmp(buf, "RESET") == 0) {
        return cmd_reset(s, out, cap);
    }

    reply(out, cap, "ERR unknown '%s'", buf);
    return false;
}
