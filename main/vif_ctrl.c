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

#include "esp_app_desc.h"
#include "esp_log.h"

#include "common.h"
#include "vif_ctrl.h"

#define TAG "vif_ctrl"

/** Longest command word we need to recognise, plus its argument. */
#define CTRL_CMD_MAX 48

static void reply(char *out, size_t cap, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void reply(char *out, size_t cap, const char *fmt, ...) {
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
static bool normalise(const char *cmd, size_t len, char *dst, size_t cap) {
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
static void mode_list(char *dst, size_t cap) {
    const vif_frontend_t *fe;
    size_t pos = 0;

    dst[0] = '\0';

    for (size_t i = 0; (fe = vif_frontend_at(i)) != NULL; i++) {
        int n =
            snprintf(dst + pos, cap - pos, "%s%s", pos ? "," : "", fe->name);

        if (n < 0 || (size_t)n >= cap - pos) {
            break;
        }
        pos += (size_t)n;
    }
}

static bool cmd_id(vif_session_t *s, char *out, size_t cap) {
    char modes[64];
    const char *fe = vif_session_frontend_name(s);

    mode_list(modes, sizeof(modes));

    reply(out, cap, "%s %s fw=%s link=%s mode=%s modes=%s", OPENDIAG_PRODUCT,
          OPENDIAG_HARDWARE, esp_app_get_description()->version,
          vif_session_name(s) ? vif_session_name(s) : "-", fe ? fe : "-",
          modes);

    return true;
}

static bool cmd_mode_set(vif_session_t *s, const char *name, char *out,
                         size_t cap) {
    const vif_frontend_t *fe = vif_frontend_find(name);
    char modes[64];

    if (!fe) {
        mode_list(modes, sizeof(modes));
        reply(out, cap, "ERR no mode '%s'; have %s", name, modes);
        return false;
    }

    /* Every claim goes with the old grammar; vif drops them on the way. */
    if (vif_session_set_frontend(s, fe) != ESP_OK) {
        reply(out, cap, "ERR cannot switch to %s", fe->name);
        return false;
    }

    ESP_LOGI(TAG, "%s: mode %s", vif_session_name(s), fe->name);

    reply(out, cap, "MODE %s", fe->name);
    return true;
}

/**
 * @brief What is live on the wire, and who holds it.
 *
 * Several buses can be up at once, so this is a list. A client refused a
 * claim has no other way to find out who has it; the shell is not reachable
 * from a phone.
 */
static bool cmd_bus(char *out, size_t cap) {
    vif_bus_claim_t claim[VIF_BUS_GROUPS];
    size_t pos = 0;

    vif_bus_info(claim);

    pos += (size_t)snprintf(out, cap, "BUS");

    for (size_t i = 0; i < VIF_BUS_GROUPS && pos < cap; i++) {
        int n;

        if (claim[i].bus == VIF_BUS_NONE) {
            continue;
        }

        n = snprintf(out + pos, cap - pos, " %s", vif_bus_name(claim[i].bus));
        if (n < 0 || (size_t)n >= cap - pos) {
            break;
        }
        pos += (size_t)n;

        if (claim[i].bitrate) {
            n = snprintf(out + pos, cap - pos, "@%" PRIu32, claim[i].bitrate);
            if (n < 0 || (size_t)n >= cap - pos) {
                break;
            }
            pos += (size_t)n;
        }

        n = snprintf(out + pos, cap - pos, "/%s", claim[i].owner);
        if (n < 0 || (size_t)n >= cap - pos) {
            break;
        }
        pos += (size_t)n;
    }

    return true;
}

static bool cmd_reset(vif_session_t *s, char *out, size_t cap) {
    const vif_frontend_t *fe = vif_session_default_frontend(s);

    /*
     * Unlike a client simply going away, this is an explicit request to let
     * go: every bus and any energised pin are released as well as the
     * grammar. Done here rather than left to the switch below, because a
     * session with no default has no switch to make.
     */
    esp_err_t err = vif_bus_release_all(s);
    vif_pin_release_all(s);

    if (err != ESP_OK) {
        reply(out, cap, "ERR bus close failed: %s", esp_err_to_name(err));
        return false;
    }
    if (fe && vif_session_set_frontend(s, fe) != ESP_OK) {
        reply(out, cap, "ERR cannot reset %s", fe->name);
        return false;
    }

    reply(out, cap, "RESET %s", fe ? fe->name : "-");
    return true;
}

bool vif_ctrl_exec(vif_session_t *s, const char *cmd, size_t len, char *out,
                   size_t cap) {
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
        /* Both forms are about a link's grammar, and the shell is not a link.
         */
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
