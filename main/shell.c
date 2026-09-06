/* SPDX-License-Identifier: GPL-3.0-only */
#include "shell.h"
#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ble_uart.h"
#include "board.h"
#include "can_bus.h"
#include "comm_iface.h"
#include "common.h"
#include "elm327_at.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline.h"
#include "link.h"
#include "slcan.h"
#include "soc/rtc_cntl_reg.h"
#include "utility.h"
#include "vif.h"
#include "vif_ctrl.h"

#define TAG "SHELL"

// --- Configuration ---
#define SHELL_TASK_PRIORITY 5
#define SHELL_TASK_STACK_SIZE 4096
#define SHELL_PROMPT "> "
/**
 * @brief Longest console line.
 *
 * Sized so a maximum length K-Line message can actually be typed: ISO 14230-2
 * clause 4.1.4 allows 260 bytes, which is 520 hex characters plus the command
 * word in front of them. The console runs on the main task, whose stack has
 * room for it.
 */
#define MAX_COMMAND_LEN 560

/* How long to wait before looking for a keypress again. Short enough that
 * typing does not feel laggy, long enough not to spin a core. */
#define SHELL_POLL_MS 10
#define MAX_ARGS 10

// --- Globals ---

// --- Type Definitions for Command Handling ---

// Forward declaration of the command function pointer type
typedef void (*command_func_t)(int argc, char **argv);

// Structure to define a shell command
typedef struct {
    const char *name;
    command_func_t func;
    const char *help;
} shell_command_t;

// --- Command Handler Functions ---

static void help_command(int argc, char **argv);

/**
 * @brief Command handler for 'heap'
 *
 * Prints the current free heap size in bytes. Useful for monitoring memory
 * usage.
 *
 * @param argc Argument count (unused)
 *param argv Argument vector (unused)
 */
static void heap_command(int argc, char **argv) {
    printf("Free Heap Size: %lu bytes\n",
           (unsigned long)esp_get_free_heap_size());
}

/**
 * @brief Command handler for 'tasks'
 *
 * Displays a table of all running FreeRTOS tasks, their states, priorities,
 * stack high water mark, and task numbers. This is an invaluable debugging
 * tool.
 *
 * @param argc Argument count (unused)
 * @param argv Argument vector (unused)
 */
static void tasks_command(int argc, char **argv) {
    // Allocate a buffer to hold the task list.
    // The buffer size needs to be sufficient to hold the entire list.
    // 40 bytes per task is a conservative estimate.
    char *task_list_buffer = malloc(uxTaskGetNumberOfTasks() * 40);
    if (task_list_buffer == NULL) {
        printf("Error: Failed to allocate memory for task list\n");
        return;
    }

    fputs("Task Name\tStatus\tPrio\tHWM\tTask#", stdout);
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
    fputs("\tAffinity", stdout);
#endif
    fputs("\n-----------------------------------------------------\n", stdout);

    // Generate the task list and print it.
    vTaskList(task_list_buffer);
    printf("%s\n", task_list_buffer);

    // Free the buffer
    free(task_list_buffer);
}

/**
 * @brief Command handler for 'reboot'
 *
 * Restarts the ESP32.
 *
 * @param argc Argument count
 * @param argv Argument vector
 */
static void reboot_command(int argc, char **argv) {
    if (argc > 1) {
        // `reboot dl` - reboot into download mode

        if (strcmp(argv[1], "dl") == 0) {
            REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
        }
    }
    printf("Rebooting...\n");
    esp_restart();
}

/*
 * Hardware commands, from here down.
 *
 * The console is not a comm_iface service - it talks to UART0 directly - but
 * it drives the same hardware as the protocol front-ends, so it holds its
 * claims the same way, as VIF_OWNER_SHELL. A pin raised here is refused to
 * ELM327, and vice versa, instead of the two silently overwriting each other.
 */

/**
 * @brief Command handler for 'hsset'
 *
 * Drives one high side pin from the boost converter. SAE J2534 allows one high
 * side pin at a time, which vif enforces, so switch the current one off before
 * moving to another.
 *
 * @param argc Argument count. Expects 3.
 * @param argv Pin number, then millivolts. 0 mV switches the pin off.
 */
static void hsset_command(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: hsset <pin> <millivolt>   (0 mV switches it off)\n");
        return;
    }

    int pin = atoi(argv[1]);
    uint32_t millivolt = (uint32_t)strtol(argv[2], NULL, 10);
    esp_err_t err;

    if (millivolt) {
        err = vif_pin_set(VIF_OWNER_SHELL, pin, VIF_PIN_VOLTAGE, millivolt);
    } else {
        err = vif_pin_set(VIF_OWNER_SHELL, pin, VIF_PIN_OFF, 0);
    }

    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
        return;
    }

    printf("OK\n");
}

/**
 * @brief Command handler for 'lsset'
 *
 * Pulls the low side pin to ground, or releases it.
 *
 * @param argc Argument count. Expects 3.
 * @param argv Pin number, then 1 to ground it and 0 to release it.
 */
static void lsset_command(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: lsset <pin> <state>\n");
        return;
    }

    int pin = atoi(argv[1]);
    int state = atoi(argv[2]);
    esp_err_t err;

    err = vif_pin_set(VIF_OWNER_SHELL, pin,
                      state ? VIF_PIN_GROUND : VIF_PIN_OFF, 0);
    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
        return;
    }

    printf("OK\n");
}

/**
 * @brief Command handler for 'pinoff'
 *
 * Releases every pin this session holds, in one go.
 */
static void pinoff_command(int argc, char **argv) {
    vif_pin_release_all(VIF_OWNER_SHELL);
    printf("OK\n");
}

/**
 * @brief Command handler for 'board'
 *
 * Display the board infomation.
 */
static void board_command(int argc, char **argv) { board_print_info(); }

/** @brief Print the embedded firmware version and build details. */
static void version_command(int argc, char **argv) {
    const esp_app_desc_t *desc = esp_app_get_description();

    printf("%s %s (%s)\n", OPENDIAG_PRODUCT, desc->version, OPENDIAG_HARDWARE);
#ifndef CONFIG_APP_REPRODUCIBLE_BUILD
    printf("\tbuilt  : %s %s\n", desc->date, desc->time);
#endif
    printf("\tidf    : %s\n", desc->idf_ver);
}

/**
 * @brief Command handler for 'vbatt'
 *
 * Display the ADC reading for battery voltage.
 */
static void vbatt_command(int argc, char **argv) {
    int32_t val = vif_vbatt_mv();
    printf("VBATT: %" PRId32 " mV\n", val);
}

/**
 * @brief Command handler for 'hsvsense'
 *
 * Display the ADC reading for high side voltage sense.
 */
static void hsvsense_command(int argc, char **argv) {
    int32_t val = vif_hs_vsense_mv();
    printf("HS VSENSE: %" PRId32 " mV\n", val);
}

static void hscal_command(int argc, char **argv) {
    esp_err_t err = vif_calibrate_hs(VIF_OWNER_SHELL);

    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
    }
}

static void vbattcal_command(int argc, char **argv) {
    esp_err_t err = vif_calibrate_vbatt();

    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
    }
}

/** @brief Parse one decimal calibration value, false if it is not a number. */
static bool calset_parse(const char *arg, int32_t *out) {
    char *end;
    long val;

    errno = 0;
    val = strtol(arg, &end, 10);
    if (*arg == '\0' || *end != '\0' || errno == ERANGE || val < INT32_MIN ||
        val > INT32_MAX) {
        printf("ERROR: '%s' is not a valid value\n", arg);
        return false;
    }

    *out = (int32_t)val;
    return true;
}

/**
 * @brief Command handler for 'calset'
 *
 * hscal and vbattcal derive the calibration constants on the bench. This
 * writes them directly instead, to put a known good set (from the 'board'
 * dump of the same unit) back on a board whose NVS was erased.
 *
 * @param argc Argument count. Expects 3, or 2 plus one value per field for
 *             the 'all' form.
 * @param argv Field name and value, or 'all' and every value in the order
 *             'board' prints them.
 */
static void calset_command(int argc, char **argv) {
    int32_t values[MAX_ARGS];
    const char *name;
    esp_err_t err;
    int nfields;
    int i;

    for (nfields = 0; board_calibration_field(nfields) != NULL; nfields++) {
        ;
    }

    if (argc >= 2 && strcmp(argv[1], "all") == 0) {
        if (nfields > (int)(sizeof(values) / sizeof(values[0])) ||
            argc != nfields + 2) {
            printf("ERROR: 'calset all' takes %d values\n", nfields);
            return;
        }

        /* Parse every value before writing any, so a typo in the last one
         * does not leave half the constants replaced. */
        for (i = 0; i < nfields; i++) {
            if (!calset_parse(argv[i + 2], &values[i])) {
                return;
            }
        }

        for (i = 0; i < nfields; i++) {
            name = board_calibration_field(i);
            err = vif_calibration_set(name, values[i]);
            if (err != ESP_OK) {
                printf("ERROR: %s: %s\n", name, esp_err_to_name(err));
                return;
            }
        }

        printf("OK\n");
        return;
    }

    if (argc == 3) {
        if (!calset_parse(argv[2], &values[0])) {
            return;
        }

        err = vif_calibration_set(argv[1], values[0]);
        if (err != ESP_OK) {
            printf("ERROR: %s\n", esp_err_to_name(err));
            return;
        }

        printf("OK\n");
        return;
    }

    printf("Usage: calset <field> <value>\n");
    printf("       calset all <value> ...   (in the order below)\n");
    printf("Fields:");
    for (i = 0; i < nfields; i++) {
        printf(" %s", board_calibration_field(i));
    }
    printf("\n");
}

/**
 * @brief Open a bus for a test command, unless this session already has it.
 */
static bool shell_bus_ready(vif_bus_t bus, const vif_bus_cfg_t *cfg) {
    esp_err_t err;

    if (vif_bus_is_open(VIF_OWNER_SHELL, bus)) {
        return true;
    }

    err = vif_bus_open(VIF_OWNER_SHELL, bus, cfg);
    if (err != ESP_OK) {
        printf("ERROR: cannot open the bus: %s\n", esp_err_to_name(err));
        return false;
    }

    return true;
}

/** Replies one bus test command will print before handing the console back. */
/** Replies one bus test command will print before handing the console back. */
#define SHELL_BUS_MAX_REPLIES 16

/** How long to wait for a first reply, and for each one after it.
 *
 *  The second number is short on purpose: a second module answering the same
 *  functional request is along within P2min of the first, so once anything has
 *  come back there is no reason to sit through another whole second before
 *  giving the console back. */
#define SHELL_BUS_FIRST_MS 1000
#define SHELL_BUS_NEXT_MS 300

/**
 * @brief Spell out the status bits that are set, or nothing at all.
 *
 * Returns a leading ", " so it can be appended unconditionally. A hex mask
 * would be shorter and would need bus.h open beside it to read, which is not
 * what somebody staring at a misbehaving vehicle wants.
 */
static const char *shell_bus_status(uint16_t status, char *buf, size_t cap) {
    static const struct {
        uint16_t bit;
        const char *name;
    } names[] = {
        {BUS_RX_BAD_CHECKSUM, "bad-checksum"},
        {BUS_RX_BUFFER_OVERFLOW, "data-lost-before-this"},
        {BUS_RX_BREAK, "after-break"},
        {BUS_RX_PERIODIC_REPLY, "answer-to-our-periodic"},
        {BUS_RX_DUPLICATE, "duplicate"},
        {BUS_RX_START_OF_MSG, "partial"},
    };
    size_t used = 0;

    buf[0] = '\0';

    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if ((status & names[i].bit) == 0) {
            continue;
        }

        int n = snprintf(buf + used, cap - used, ", %s", names[i].name);
        if (n < 0 || (size_t)n >= cap - used) {
            break;
        }
        used += (size_t)n;
    }

    return buf;
}

/** @brief Print one message the way a person reads it: bytes first. */
static void shell_bus_print_msg(const bus_msg_t *msg, int64_t sent_us) {
    bool ours = (msg->status & BUS_RX_TX_MSG_TYPE) != 0;
    char flags[96];

    printf("%s ", ours ? "tx" : "rx");

    for (uint16_t i = 0; i < msg->len; i++) {
        printf("%02X ", msg->data[i]);
    }

    printf(" (%u bytes", msg->len);

    if (!ours) {
        /* Milliseconds from the request being issued to this reply being
         * complete. It covers any quiet time the protocol owed before the
         * request could go out, the request itself, and the module's own
         * latency - so it is the cost of the exchange rather than P2 alone.
         * "kline stats" reports the measured P2 separately.
         *
         * An absolute free-running timestamp says nothing without a second
         * one to subtract it from, and on our own loopback copy there is
         * nothing to subtract, so it is left off rather than printed as a
         * rounding error either side of zero. */
        uint32_t age_us = (uint32_t)msg->timestamp_us - (uint32_t)sent_us;

        printf(", %.1f ms", age_us / 1000.0);
    }

    printf("%s)\n", shell_bus_status(msg->status, flags, sizeof(flags)));
}

/**
 * @brief Send a hex string on a byte bus and print whatever answers.
 *
 * Shared by the kline, j1850 and vpw commands, which differ only in which bus
 * they ask vif for. The reply count is capped as well as the idle timeout: a
 * J1850 VPW bus carries periodic broadcasts, and waiting for a gap in those
 * would keep the console for as long as the vehicle is awake.
 *
 * The output is command output, not logging: no tag, no timestamp column, and
 * the bytes on the same line as what is said about them. Running out of the
 * reply window is how every exchange ends and is not worth a line of its own -
 * the count at the bottom already says how many came back.
 */
static void shell_bus_command(vif_bus_t bus, const char *name, int argc,
                              char **argv) {
    uint8_t data[KLINE_MAX_MSG];
    bus_msg_t msg;
    int64_t sent_us;
    int replies = 0;
    int rc;

    if (argc != 2) {
        printf("Usage: %s <hex string>|stop\n", name);
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        esp_err_t err = vif_bus_close(VIF_OWNER_SHELL, bus);
        if (err != ESP_OK) {
            printf("ERROR: %s\n", esp_err_to_name(err));
        } else {
            printf("OK\n");
        }
        return;
    }

    size_t len = strlen(argv[1]);
    if (len > sizeof(data) * 2) {
        printf("hex string too long\n");
        return;
    }

    rc = hex_string_to_u8_array(argv[1], len, data, sizeof(data));
    if (rc <= 0) {
        printf("not hex: %s\n", argv[1]);
        return;
    }

    if (!shell_bus_ready(bus, NULL)) {
        return;
    }

    /* Before the send, not after it. What send() waits for before returning
     * is the driver's business and differs between buses - J1850 lingers to
     * find out whether anything answered, which means a reply can be
     * timestamped by the receive interrupt before the send call has even
     * come back. Anchoring here makes the figure the honest one either way:
     * how long the whole request took, from asking to holding the answer. */
    sent_us = esp_timer_get_time();
    bus_msg_tx(&msg, data, (size_t)rc, 0);
    rc = vif_bus_send(VIF_OWNER_SHELL, bus, &msg, 0);
    if (rc != 0) {
        printf("send failed: %d\n", rc);
        return;
    }

    bus_msg_init(&msg, data, sizeof(data));

    for (int i = 0; i < SHELL_BUS_MAX_REPLIES; i++) {
        uint32_t wait = replies ? SHELL_BUS_NEXT_MS : SHELL_BUS_FIRST_MS;
        int ret = vif_bus_recv(VIF_OWNER_SHELL, bus, &msg, pdMS_TO_TICKS(wait));

        if (ret > 0) {
            shell_bus_print_msg(&msg, sent_us);
            /* A loopback copy of our own request is not an answer to it. */
            if ((msg.status & BUS_RX_TX_MSG_TYPE) == 0) {
                replies++;
            }
        } else if (ret == BUS_ERR_TIMEOUT) {
            break;
        } else {
            printf("receive failed: %d\n", ret);
            break;
        }
    }

    if (replies == 0) {
        printf("no reply within %d ms\n", SHELL_BUS_FIRST_MS);
    } else {
        printf("%d repl%s\n", replies, replies == 1 ? "y" : "ies");
    }
}

/**
 * @brief Time a run of exchanges without the console in the way.
 *
 * A throughput figure taken from outside the device measures the console: the
 * command has to be typed, echoed, parsed, and the reply window waited out,
 * and all of that is an order of magnitude larger than the exchange it is
 * supposed to be timing. So the loop runs here, and only the summary is
 * printed.
 *
 * What it reports is the time from the request starting to go out to the
 * first reply being complete - which is the whole of what the bus costs: the
 * quiet time owed before the request, the request itself at P4 a byte, the
 * module's P2, and the reply.
 */
static void shell_bus_bench(vif_bus_t bus, int rounds, const uint8_t *req,
                            size_t req_len) {
    uint8_t buf[KLINE_MAX_MSG];
    bus_msg_t msg;
    bus_msg_t tx;
    int64_t total = 0, worst = 0, best = INT64_MAX;
    int64_t started, elapsed;
    int replies = 0;

    if (rounds < 1 || rounds > 10000) {
        printf("rounds out of range\n");
        return;
    }
    if (!shell_bus_ready(bus, NULL)) {
        return;
    }

    bus_msg_init(&msg, buf, sizeof(buf));
    started = esp_timer_get_time();

    for (int i = 0; i < rounds; i++) {
        int64_t t0 = esp_timer_get_time();
        int64_t took;

        /* One mark per exchange, wrapped. A long run is minutes of an
         * otherwise silent console, and anything reading this over a socket
         * has no way to tell that from a device that has stopped. */
        printf(".");
        if ((i % 50) == 49) {
            printf("\n");
        }
        fflush(stdout);

        bus_msg_tx(&tx, req, req_len, 0);
        if (vif_bus_send(VIF_OWNER_SHELL, bus, &tx, 0) != 0) {
            printf("send failed at round %d\n", i);
            break;
        }

        /* Long enough for any legal P2max and then some, short enough that a
         * module that has stopped answering does not dominate the average. */
        if (vif_bus_recv(VIF_OWNER_SHELL, bus, &msg, pdMS_TO_TICKS(250)) <= 0) {
            continue;
        }

        took = esp_timer_get_time() - t0;
        total += took;
        replies++;
        if (took > worst) {
            worst = took;
        }
        if (took < best) {
            best = took;
        }
    }

    elapsed = esp_timer_get_time() - started;

    printf("\nbus-bench\n");
    printf("  rounds           %d\n", rounds);
    printf("  replies          %d\n", replies);
    printf("  elapsed-ms       %" PRId64 "\n", elapsed / 1000);
    printf("  request-bytes    %u\n", (unsigned)req_len);

    if (replies) {
        printf("  reply-bytes      %u\n", msg.len);
        printf("  mean-us          %" PRId64 "\n", total / replies);
        printf("  min-us           %" PRId64 "\n", best);
        printf("  max-us           %" PRId64 "\n", worst);
        printf("  exchanges-per-s  %" PRId64 "\n",
               elapsed ? (int64_t)replies * 1000000 / elapsed : 0);
    }
}

/**
 * @brief Print the shared byte bus counters.
 *
 * One counter per line, name then value, names unique across the block - the
 * target test suites parse this, and two different counters sharing a name is
 * a test that quietly measures the wrong thing.
 *
 * The counters are split by failure mode on purpose. A bus with nothing on
 * it, a vehicle that wants 9600 baud, and a marginal ground each look
 * completely different here and identical in a single error total.
 */
static void bus_print_stats(vif_bus_t bus, const char *label) {
    bus_stats_t s;

    if (vif_bus_stats(bus, &s) != 0) {
        printf("%s has no counters\n", label);
        return;
    }

    printf("%s-counters\n", label);
    printf("  rx-msgs          %" PRIu32 "\n", s.rx_msgs);
    printf("  rx-bytes         %" PRIu32 "\n", s.rx_bytes);
    printf("  rx-bad-checksum  %" PRIu32 "\n", s.rx_bad_checksum);
    printf("  rx-short         %" PRIu32 "\n", s.rx_short);
    printf("  rx-too-long      %" PRIu32 "\n", s.rx_too_long);
    printf("  rx-dropped       %" PRIu32 "\n", s.rx_dropped);
    printf("  rx-overrun       %" PRIu32 "\n", s.rx_overrun);
    printf("  rx-break         %" PRIu32 "\n", s.rx_break);
    printf("  rx-frame-err     %" PRIu32 "\n", s.rx_frame_err);
    printf("  rx-suppressed    %" PRIu32 "\n", s.rx_suppressed);
    printf("  rx-duplicate     %" PRIu32 "\n", s.rx_duplicate);
    printf("  tx-msgs          %" PRIu32 "\n", s.tx_msgs);
    printf("  tx-bytes         %" PRIu32 "\n", s.tx_bytes);
    printf("  tx-echo-ok       %" PRIu32 "\n", s.tx_echo_ok);
    printf("  tx-echo-missing  %" PRIu32 "\n", s.tx_echo_missing);
    printf("  tx-echo-bad      %" PRIu32 "\n", s.tx_echo_bad);
    printf("  tx-retries       %" PRIu32 "\n", s.tx_retries);
    printf("  tx-bus-busy      %" PRIu32 "\n", s.tx_bus_busy);
    printf("  init-attempts    %" PRIu32 "\n", s.init_attempts);
    printf("  init-ok          %" PRIu32 "\n", s.init_ok);
    printf("  init-no-sync     %" PRIu32 "\n", s.init_no_sync);
    printf("  init-no-keys     %" PRIu32 "\n", s.init_no_keys);
    printf("  init-bad-keys    %" PRIu32 "\n", s.init_bad_keys);
    printf("  init-no-addr     %" PRIu32 "\n", s.init_no_addr);
    printf("  periodic-sent    %" PRIu32 "\n", s.periodic_sent);
    /* ISO 14230-2 Table 6 and clause 4.4, as this vehicle actually performs
     * them. An ECU sitting at the edge of its W1 window is one cold start
     * from failing, and no error counter can say that. */
    printf("  last-w1-us       %" PRIu32 "\n", s.last_w1_us);
    printf("  last-w2-us       %" PRIu32 "\n", s.last_w2_us);
    printf("  last-w3-us       %" PRIu32 "\n", s.last_w3_us);
    printf("  last-w4-us       %" PRIu32 "\n", s.last_w4_us);
    printf("  last-p2-us       %" PRIu32 "\n", s.last_p2_us);
    printf("  last-p1-max-us   %" PRIu32 "\n", s.last_p1_max_us);
}

/**
 * @brief Print every parameter the bus in use will answer for.
 *
 * A bus answers BUS_ERR_UNSUPPORTED for the ones that do not apply to it,
 * so this walks the whole table and prints what comes back. That is also the
 * quickest way to see what a driver actually implements, which is worth
 * having when the same command works on three different buses.
 */
static void bus_print_params(vif_bus_t bus) {
    bus_param_t p;
    const char *name;
    size_t i = 0;

    printf("bus-parameters\n");

    while ((name = bus_param_at(i++, &p)) != NULL) {
        uint32_t value;

        if (vif_bus_param_get(VIF_OWNER_SHELL, bus, p, &value) == 0) {
            printf("  %-22s %" PRIu32 "\n", name, value);
        }
    }
}

/**
 * @brief Set one parameter by the name bus_print_params() prints.
 *
 * The same command on every bus, because the parameter identifiers are
 * J2534's and the drivers answer to them directly - which is the point of
 * the shared interface and the quickest way to prove it works.
 */
static void bus_set_param(vif_bus_t bus, const char *name, const char *value) {
    bus_param_t p;
    int rc;

    if (!bus_param_from_name(name, &p)) {
        printf("no parameter called \"%s\"\n", name);
        return;
    }

    rc = vif_bus_param_set(VIF_OWNER_SHELL, bus, p,
                           (uint32_t)strtoul(value, NULL, 0));
    if (rc == BUS_ERR_UNSUPPORTED) {
        printf("this bus has no %s\n", name);
    } else if (rc != 0) {
        printf("set %s failed: %d\n", name, rc);
    } else {
        printf("OK\n");
    }
}

/** @brief Print what the last initialisation established. */
static void kline_print_link(void) {
    bus_link_t l;
    kline_keybytes_t k;

    if (vif_bus_ioctl(VIF_OWNER_SHELL, VIF_BUS_KLINE, BUS_IOCTL_GET_LINK, NULL,
                      &l) != 0) {
        printf("no K-Line bus\n");
        return;
    }

    kline_decode_keybytes(l.key[0], l.key[1], &k);

    printf("kline-link\n");
    printf("  connected        %d\n", l.connected ? 1 : 0);
    printf("  variant          %s\n",
           kline_variant_name((kline_variant_t)l.variant));
    printf("  baud             %" PRIu32 "\n", l.baud);
    printf("  key-bytes        %02X %02X\n", l.key[0], l.key[1]);
    printf("  key-code         %u\n", l.key_code);
    printf("  key-parity       %d\n", k.parity_ok ? 1 : 0);
    printf("  hdr-address      %d\n", k.hdr_address ? 1 : 0);
    printf("  hdr-1byte        %d\n", k.hdr_1byte ? 1 : 0);
    printf("  len-in-format    %d\n", k.len_in_format ? 1 : 0);
    printf("  extra-len-byte   %d\n", k.extra_len_byte ? 1 : 0);
    printf("  extended-timing  %d\n", k.extended_timing ? 1 : 0);
    printf("  init-address     %02X\n", l.address);
    printf("  w1-us            %" PRIu32 "\n", l.w1_us);
    printf("  w2-us            %" PRIu32 "\n", l.w2_us);
    printf("  w3-us            %" PRIu32 "\n", l.w3_us);
    printf("  w4-us            %" PRIu32 "\n", l.w4_us);
}

/**
 * @brief Run one initialisation from the console.
 *
 * Named modes rather than protocol numbers, because on this bus the protocol
 * is an *outcome* of the handshake: a slow init tells you whether the vehicle
 * is ISO 9141-2 or ISO 14230-4 from the key bytes it answers with. Use
 * "kline link" afterwards to see which it was.
 */
static void kline_init_command(const char *mode, const char *addr) {
    bus_init_t io;
    bus_ioctl_t which;
    int rc;

    memset(&io, 0, sizeof(io));
    io.address = KLINE_INIT_ADDR_OBD;

    if (mode == NULL || strcmp(mode, "slow") == 0) {
        which = BUS_IOCTL_FIVE_BAUD_INIT;
    } else if (strcmp(mode, "fast") == 0) {
        which = BUS_IOCTL_FAST_INIT;
        io.msg[0] = 0xC1;
        io.msg[1] = KLINE_INIT_ADDR_OBD;
        io.msg[2] = 0xF1;
        io.msg[3] = KLINE_SVC_START_COMM;
        io.msg_len = 4;
    } else if (strcmp(mode, "none") == 0) {
        /* Bring the bus up and claim it, without touching the wire. */
        if (!shell_bus_ready(VIF_BUS_KLINE, NULL)) {
            return;
        }
        printf("OK\n");
        return;
    } else {
        printf("Usage: kline init [slow|fast|none] [address]\n");
        return;
    }

    if (addr) {
        uint8_t a[1];

        if (hex_string_to_u8_array(addr, strlen(addr), a, sizeof(a)) != 1) {
            printf("bad init address\n");
            return;
        }
        io.address = a[0];
        if (io.msg_len) {
            io.msg[1] = a[0];
        }
    }

    if (!shell_bus_ready(VIF_BUS_KLINE, NULL)) {
        return;
    }

    rc = vif_bus_ioctl(VIF_OWNER_SHELL, VIF_BUS_KLINE, which, &io, &io);
    if (rc != 0) {
        printf("init failed: %d\n", rc);
        return;
    }

    kline_print_link();
    printf("OK\n");
}

static void kline_command(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "stats") == 0) {
        bus_print_stats(VIF_BUS_KLINE, "kline");
        return;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        vif_bus_reset_stats(VIF_BUS_KLINE);
        printf("OK\n");
        return;
    }
    if (argc == 2 && strcmp(argv[1], "link") == 0) {
        kline_print_link();
        return;
    }
    if ((argc == 3 || argc == 4) && strcmp(argv[1], "bench") == 0) {
        /* Default to the mode 01 PID 00 request every OBD-II client makes
         * first, so a bare "kline bench 50" measures something meaningful. */
        static const uint8_t def[] = {0x68, 0x6A, 0xF1, 0x01, 0x00};
        uint8_t req[32];
        int n = (int)sizeof(def);

        if (argc == 4) {
            n = hex_string_to_u8_array(argv[3], strlen(argv[3]), req,
                                       sizeof(req));
            if (n <= 0) {
                printf("not hex: %s\n", argv[3]);
                return;
            }
        } else {
            memcpy(req, def, sizeof(def));
        }

        shell_bus_bench(VIF_BUS_KLINE, atoi(argv[2]), req, (size_t)n);
        return;
    }
    if (argc >= 2 && strcmp(argv[1], "init") == 0) {
        kline_init_command(argc > 2 ? argv[2] : NULL,
                           argc > 3 ? argv[3] : NULL);
        return;
    }
    if (argc == 2 && strcmp(argv[1], "close") == 0) {
        printf("%s\n", vif_bus_ioctl(VIF_OWNER_SHELL, VIF_BUS_KLINE,
                                     BUS_IOCTL_STOP_COMM, NULL, NULL) == 0
                           ? "OK"
                           : "ERROR");
        return;
    }
    if (argc == 2 && strcmp(argv[1], "cfg") == 0) {
        bus_print_params(VIF_BUS_KLINE);
        return;
    }
    if (argc == 4 && strcmp(argv[1], "cfg") == 0) {
        if (!shell_bus_ready(VIF_BUS_KLINE, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_KLINE, argv[2], argv[3]);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "wm") == 0) {
        uint8_t buf[BUS_PERIODIC_MAX];
        bus_msg_t msg;
        int n =
            hex_string_to_u8_array(argv[2], strlen(argv[2]), buf, sizeof(buf));

        if (n <= 0) {
            printf("bad periodic message\n");
            return;
        }
        if (!shell_bus_ready(VIF_BUS_KLINE, NULL)) {
            return;
        }
        bus_msg_init(&msg, buf, (size_t)n);
        msg.len = (uint16_t)n;
        printf("%s\n", vif_bus_ioctl(VIF_OWNER_SHELL, VIF_BUS_KLINE,
                                     BUS_IOCTL_SET_PERIODIC, &msg, NULL) == 0
                           ? "OK"
                           : "ERROR");
        return;
    }

    shell_bus_command(VIF_BUS_KLINE, "kline", argc, argv);
}

/**
 * @brief Print the counters only J1850 PWM has.
 *
 * The shared ones come from bus_print_stats(); these are the symbol level
 * failures that exist on no other bus. Folding them into the shared names
 * would produce counters that read the same and mean different things
 * depending on which driver answered, so they are printed alongside instead.
 *
 * One counter per line, name then value, names unique across both blocks -
 * the target suite parses this, and an ambiguous name is a test that quietly
 * measures the wrong thing.
 */
static void j1850_print_stats(void) {
    j1850_pwm_stats_t s;

    bus_print_stats(VIF_BUS_J1850_PWM, "j1850-pwm");

    j1850_pwm_get_native_stats(&s);

    printf("j1850-pwm-symbols\n");
    printf("  rx-ifr           %" PRIu32 "\n", s.rx_ifr);
    printf("  rx-bad-crc       %" PRIu32 "\n", s.rx_bad_crc);
    printf("  rx-framing       %" PRIu32 "\n", s.rx_framing);
    printf("  rx-bad-symbol    %" PRIu32 "\n", s.rx_bad_symbol);
    printf("  rx-bad-timing    %" PRIu32 "\n", s.rx_bad_timing);
    printf("  rx-no-sof        %" PRIu32 "\n", s.rx_no_sof);
    printf("  rx-empty         %" PRIu32 "\n", s.rx_empty);
    printf("  rx-isr-worst-us  %" PRIu32 "\n", s.rx_isr_us_max);
    printf("  ifr-sent         %" PRIu32 "\n", s.ifr_sent);
    printf("  ifr-sof          %" PRIu32 "\n", s.ifr_sof);
    printf("  ifr-candidates   %" PRIu32 "\n", s.ifr_candidates);
    printf("  ifr-bad-pulse    %" PRIu32 "\n", s.ifr_bad_pulse);
    printf("  ifr-bad-gap      %" PRIu32 "\n", s.ifr_bad_gap);
    printf("  ifr-late         %" PRIu32 "\n", s.ifr_late);
    printf("  ifr-lost         %" PRIu32 "\n", s.ifr_lost);
    printf("  ifr-observed     %" PRIu32 "\n", s.ifr_observed);
    printf("  ifr-gap-min-us   %" PRIu32 "\n", s.ifr_gap_min_us);
    printf("  ifr-gap-max-us   %" PRIu32 "\n", s.ifr_gap_max_us);
    /* Excludes interrupt latency; a value inside Tp4 is not proof that
     * the waveform meets its 42..54 us receive window. */
    printf("  ifr-start-us     %" PRIu32 "\n", s.ifr_start_us);
}

/**
 * @brief Print the pulse train behind the last thing the receiver saw.
 *
 * Widths, not bytes. A transceiver whose turn-on and turn-off delays differ
 * skews every active pulse the same way, and seeing Tp1 sitting at 9 us
 * instead of 7 is how that gets caught before it starts costing frames.
 */
static void j1850_print_capture(j1850_pwm_capture_sel_t which) {
    rmt_symbol_word_t sym[J1850_PWM_CAPTURE_MAX];
    j1850_pwm_rx_status_t st;
    size_t n =
        j1850_pwm_get_capture(which, sym, sizeof(sym) / sizeof(sym[0]), &st);

    if (n == 0) {
        printf("nothing captured yet\n");
        return;
    }

    printf("%u symbols, decoded as \"%s\"\n", (unsigned)n,
           j1850_pwm_rx_status_str(st));

    for (size_t i = 0; i < n; i++) {
        printf("  %3u: %s %3u us / %s %3u us\n", (unsigned)i,
               sym[i].level0 ? "act" : "psv", sym[i].duration0,
               sym[i].level1 ? "act" : "psv", sym[i].duration1);
    }
}

static void j1850_command(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "stats") == 0) {
        j1850_print_stats();
        return;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        vif_bus_reset_stats(VIF_BUS_J1850_PWM);
        printf("OK\n");
        return;
    }
    if (argc == 2 && strcmp(argv[1], "cfg") == 0) {
        bus_print_params(VIF_BUS_J1850_PWM);
        return;
    }
    if (argc == 4 && strcmp(argv[1], "cfg") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_PWM, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_PWM, argv[2], argv[3]);
        return;
    }
    if (argc == 2 && strcmp(argv[1], "dump") == 0) {
        j1850_print_capture(J1850_PWM_CAP_LAST);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "dump") == 0 &&
        strcmp(argv[2], "tx") == 0) {
        /* This node's own last frame, as its own receiver saw it: the
         * measurement the transmit trim is derived from. */
        j1850_print_capture(J1850_PWM_CAP_TX_ECHO);
        return;
    }
    /* dup, retries and ifr are parameters like any other now; these three
     * spellings stay because the target suite and a decade of muscle memory
     * use them. */
    if (argc == 3 && strcmp(argv[1], "dup") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_PWM, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_PWM, "duplicate-ms", argv[2]);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "retries") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_PWM, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_PWM, "tx-retries", argv[2]);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "ifr") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_PWM, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_PWM, "ifr-enabled", argv[2]);
        return;
    }

    shell_bus_command(VIF_BUS_J1850_PWM, "j1850", argc, argv);
}

/**
 * @brief "vpw stats": the shared counters and this driver's own beside them.
 *
 * Same shape as j1850_print_stats() and the same reasoning: the symbol level
 * failures have no equivalent on any other bus, and giving them shared names
 * would produce counters that read the same and mean different things
 * depending on which driver answered.
 *
 * One counter per line, name then value, names unique across both blocks -
 * the target suite parses this, and an ambiguous name is a test that quietly
 * measures the wrong thing.
 */
static void vpw_print_stats(void) {
    j1850_vpw_stats_t s;

    bus_print_stats(VIF_BUS_J1850_VPW, "j1850-vpw");

    j1850_vpw_get_native_stats(&s);

    printf("j1850-vpw-symbols\n");
    printf("  rx-ifr           %" PRIu32 "\n", s.rx_ifr);
    printf("  rx-bad-crc       %" PRIu32 "\n", s.rx_bad_crc);
    printf("  rx-framing       %" PRIu32 "\n", s.rx_framing);
    printf("  rx-bad-symbol    %" PRIu32 "\n", s.rx_bad_symbol);
    printf("  rx-no-sof        %" PRIu32 "\n", s.rx_no_sof);
    printf("  rx-empty         %" PRIu32 "\n", s.rx_empty);
    printf("  rx-isr-worst-us  %" PRIu32 "\n", s.rx_isr_us_max);
    /* Clause 6.7.2 contention. "checks" is how many times the monitor
     * actually looked at the bus, which is what says it was running at all -
     * a zero there and a zero beside it mean very different things. */
    printf("  tx-arb-lost      %" PRIu32 "\n", s.tx_arb_lost);
    printf("  tx-arb-checks    %" PRIu32 "\n", s.tx_arb_checks);
    printf("  tx-arb-late      %" PRIu32 "\n", s.tx_arb_late);
    printf("  tx-arb-unsynced  %" PRIu32 "\n", s.tx_arb_unsynced);
    printf("  tx-arb-pulse     %" PRIu32 "\n", s.tx_arb_pulse);
}

/**
 * @brief Print the pulse train behind the last thing the receiver saw.
 *
 * Widths, not bytes. Table 5 allows a short pulse anywhere from 49 to 79 us,
 * so a driver stage that has started skewing every active pulse has a long
 * way to drift before it costs a frame - and this is where that shows up
 * while there is still margin left to lose.
 */
static void vpw_print_capture(j1850_vpw_capture_sel_t which) {
    rmt_symbol_word_t sym[J1850_VPW_CAPTURE_MAX];
    j1850_vpw_rx_status_t st;
    size_t n =
        j1850_vpw_get_capture(which, sym, sizeof(sym) / sizeof(sym[0]), &st);

    if (n == 0) {
        printf("nothing captured yet\n");
        return;
    }

    printf("%u symbols, decoded as \"%s\"\n", (unsigned)n,
           j1850_vpw_rx_status_str(st));

    for (size_t i = 0; i < n; i++) {
        printf("  %3u: %s %3u us / %s %3u us\n", (unsigned)i,
               sym[i].level0 ? "act" : "psv", sym[i].duration0,
               sym[i].level1 ? "act" : "psv", sym[i].duration1);
    }
}

static void j1850_vpw_command(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "stats") == 0) {
        vpw_print_stats();
        return;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        vif_bus_reset_stats(VIF_BUS_J1850_VPW);
        printf("OK\n");
        return;
    }
    if (argc == 2 && strcmp(argv[1], "cfg") == 0) {
        bus_print_params(VIF_BUS_J1850_VPW);
        return;
    }
    if (argc == 4 && strcmp(argv[1], "cfg") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_VPW, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_VPW, argv[2], argv[3]);
        return;
    }
    if (argc == 2 && strcmp(argv[1], "dump") == 0) {
        vpw_print_capture(J1850_VPW_CAP_LAST);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "dump") == 0 &&
        strcmp(argv[2], "tx") == 0) {
        /* This node's own last frame, as its own receiver saw it: the
         * measurement the target suite checks Table 5 against. */
        vpw_print_capture(J1850_VPW_CAP_TX_ECHO);
        return;
    }
    /* dup and retries are parameters like any other now; these two spellings
     * stay because the target suite and the j1850 command both use them. */
    if (argc == 3 && strcmp(argv[1], "dup") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_VPW, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_VPW, "duplicate-ms", argv[2]);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "retries") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_VPW, NULL)) {
            return;
        }
        bus_set_param(VIF_BUS_J1850_VPW, "tx-retries", argv[2]);
        return;
    }
    if (argc == 3 && strcmp(argv[1], "arbfault") == 0) {
        if (!shell_bus_ready(VIF_BUS_J1850_VPW, NULL)) {
            return;
        }
        /* Spent on the next frame the shell sends, whichever that is. */
        j1850_vpw_inject_arbitration_loss((uint32_t)strtoul(argv[2], NULL, 0));
        printf("OK\n");
        return;
    }
    if (argc == 3 && strcmp(argv[1], "collide") == 0) {
        uint8_t data[J1850_VPW_MAX_FRAME];
        size_t len = strlen(argv[2]);
        int rc;

        if (!shell_bus_ready(VIF_BUS_J1850_VPW, NULL)) {
            return;
        }
        rc = hex_string_to_u8_array(argv[2], len, data, sizeof(data));
        if (rc <= 0) {
            printf("not hex: %s\n", argv[2]);
            return;
        }

        /* Waits for another node to start, then transmits on top of it. The
         * expected outcome is a refusal: -11 is the monitor doing its job. */
        rc = j1850_vpw_force_collision(data, (size_t)rc);
        printf("collide: %d (%s)\n", rc,
               rc == BUS_ERR_ARBITRATION ? "lost the bus, stopped - as intended"
               : rc == 0                 ? "won the bus outright"
               : rc == BUS_ERR_TIMEOUT
                   ? "nothing was transmitting to collide with"
                   : "failed");
        return;
    }

    shell_bus_command(VIF_BUS_J1850_VPW, "vpw", argc, argv);
}

/**
 * @brief Command handler for 'can'
 *
 * Brings the CAN driver up and down repeatedly, which is how the teardown path
 * gets exercised on hardware.
 */
static void can_command(int argc, char **argv) {
    vif_bus_cfg_t cfg = {.bitrate = 500000};
    int cycles = 1000;

    if (argc > 1 && strcmp(argv[1], "stats") == 0) {
        /* The controller is one peripheral whoever opened it, so this reports
         * on a bus another link is using - which is the only time the error
         * counters are interesting. A test driving ELM327 over USB has no
         * other way to see that a protocol it is switching between left the
         * controller error passive. */
        can_print_stat();
        return;
    }

    if (argc > 1) {
        cycles = atoi(argv[1]);
    }

    for (int i = 0; i < cycles; i++) {
        printf("Starting CAN bus test... (%d/%d)\n", i + 1, cycles);
        if (vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg) != ESP_OK) {
            printf("ERROR: cannot open the CAN bus\n");
            return;
        }
        printf("Tearing down CAN bus...\n");
        vif_bus_close(VIF_OWNER_SHELL, VIF_BUS_CAN);
    }
}

/**
 * @brief Command handler for 'ble'
 *
 * The BLE link's access control, from the one interface that does not go over
 * BLE. A phone can be invited, and every phone can be forgotten - which is
 * the way back when a unit changes hands, and the only way to revoke a phone
 * that has already bonded.
 */
static void ble_command(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "pair") == 0) {
        ble_uart_open_pairing_window();
        printf("Pairing window open. Connect the phone now.\n");
        return;
    }

    if (argc > 1 && strcmp(argv[1], "forget") == 0) {
        ble_uart_forget_bonds();
        printf("Clearing every bond. The next phone to pair has to be "
               "invited; run 'ble' to see it done.\n");
        return;
    }

    printf("BLE link\n");
    printf("  bonded phones: %d\n", ble_uart_bond_count());
    printf("  pairing window: %s\n",
           ble_uart_pairing_window_open() ? "OPEN" : "closed");
    printf("  advertising: %s\n", ble_adv_state_name(ble_uart_adv_state()));
    printf("  connected: %s\n", ble_uart_is_connected() ? "yes" : "no");
}

/**
 * @brief Command handler for 'vif'
 *
 * Prints the sessions, their front-ends and their claims. Switching a link's
 * grammar is 'mode', which goes through the same control plane a client uses.
 */
static void vif_command(int argc, char **argv) { vif_print_debug_info(); }

/**
 * @brief Command handler for 'link'
 *
 * Which transport carries the data link, and the override for it. The rule is
 * first connected wins, which on a developer's bench means USB always: the
 * host holds CDC0 open for the whole session, so a phone connects, writes,
 * and is answered by nobody. This is how the link gets handed over.
 */
static void link_command(int argc, char **argv) {
    static const link_transport_t all[] = {LINK_USB, LINK_BLE};
    link_transport_t want;
    size_t i;

    if (argc == 1) {
        printf("link: %s\n", link_transport_name(link_holder()));
        for (i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
            printf("  %-4s connected:%-3s %s\n", link_transport_name(all[i]),
                   link_transport_connected(all[i]) ? "yes" : "no",
                   link_holder() == all[i] ? "<- holds the link" : "");
        }
        return;
    }

    if (argc != 2) {
        printf("Usage: link [drop|usb|ble]\n");
        return;
    }

    if (strcmp(argv[1], "drop") == 0) {
        want = LINK_NONE;
    } else if (strcmp(argv[1], "usb") == 0) {
        want = LINK_USB;
    } else if (strcmp(argv[1], "ble") == 0) {
        want = LINK_BLE;
    } else {
        printf("Usage: link [drop|usb|ble]\n");
        return;
    }

    if (want != LINK_NONE && !link_transport_connected(want)) {
        /* Allowed: it binds now and answers whenever the client turns up. */
        printf("note: %s has no client attached\n", link_transport_name(want));
    }

    link_set(want);
    printf("link: %s\n", link_transport_name(link_holder()));
}

/**
 * @brief Command handler for 'mode'
 *
 * The USB half of the control plane. There is one data link, so this needs no
 * argument beyond the grammar to put on it:
 *
 *   mode          what it is running, what it falls back to, what it could run
 *   mode <name>   switch it, releasing every claim it held
 *
 * The switch itself is vif_ctrl_exec(), so the shell and a BLE client cannot
 * disagree about what a mode change does.
 */
static void mode_command(int argc, char **argv) {
    char out[VIF_CTRL_REPLY_MAX];
    char cmd[VIF_CTRL_REPLY_MAX];

    if (argc == 1) {
        const vif_frontend_t *fe = vif_link_default_frontend();
        const char *now = vif_link_frontend_name();

        printf("Link: %s (default %s)\n", now ? now : "-", fe ? fe->name : "-");

        printf("Modes:");
        for (size_t i = 0; (fe = vif_frontend_at(i)) != NULL; i++) {
            printf(" %s", fe->name);
        }
        printf("\n");
        return;
    }

    if (argc != 2) {
        printf("Usage: mode [<name>]\n");
        return;
    }

    snprintf(cmd, sizeof(cmd), "MODE=%s", argv[1]);
    vif_ctrl_exec(cmd, strlen(cmd), out, sizeof(out));
    printf("%s\n", out);
}

/**
 * @brief Command handler for 'ctrl'
 *
 * Runs a raw control command against the link - the same text a BLE client
 * writes to the control characteristic. Useful for trying one without a phone.
 */
static void ctrl_command(int argc, char **argv) {
    char out[VIF_CTRL_REPLY_MAX];

    if (argc != 2) {
        printf("Usage: ctrl <ID|MODE|MODE=name|BUS|RESET>\n");
        return;
    }

    vif_ctrl_exec(argv[1], strlen(argv[1]), out, sizeof(out));
    printf("%s\n", out);
}

/**
 * @brief Command handler for 'comm'
 *
 * Shows the registered comm_iface ports and their state.
 */
static void comm_command(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "trace") == 0) {
        ble_uart_print_trace();
        return;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        ble_uart_trace_reset();
        printf("OK\n");
        return;
    }

    comm_print_debug_info();
}

const shell_command_t commands[] = {
    {"help", help_command, "Show this help message"},
    {"heap", heap_command, "Show free heap size"},
    {"tasks", tasks_command, "Show list of running tasks"},
    {"reboot", reboot_command, "Reboot the device"},
    {"hsset", hsset_command,
     "Drive a high side pin. Usage: hsset <pin> <millivolt>"},
    {"lsset", lsset_command,
     "Ground the low side pin. Usage: lsset <pin> <state>"},
    {"pinoff", pinoff_command, "Release every pin this console holds"},
    {"board", board_command, "Show board info"},
    {"version", version_command, "Show firmware version and build info"},
    {"vbatt", vbatt_command, "Show Battery Voltage"},
    {"hsvsense", hsvsense_command, "Show high side voltage sense"},
    {"hscal", hscal_command, "HS voltage calibration"},
    {"vbattcal", vbattcal_command, "VBatt voltage calibration"},
    {"calset", calset_command,
     "Write calibration constants. Usage: calset <field> <value>|all <value> "
     "..."},
    {"kline", kline_command,
     "K-Line. Usage: kline <hex>|stop|stats|clear|link|close|cfg [<name> "
     "<value>]"
     "|init [slow|fast|none] [addr]|wm <hex>|bench <rounds> [hex]"},
    {"j1850", j1850_command,
     "J1850 PWM. Usage: j1850 <hex>|stop|stats|clear|cfg [<name> <value>]|dump "
     "[tx]|retries <n>|ifr <0|1>|dup <ms>"},
    {"vpw", j1850_vpw_command,
     "J1850 VPW. Usage: vpw <hex>|stop|stats|clear|cfg [<name> <value>]|dump "
     "[tx]|retries <n>|dup <ms>|collide <hex>|arbfault <n>"},
    {"can", can_command,
     "CAN. Usage: can [cycles] - bring up/tear down loop; can stats - "
     "controller counters"},
    {"ble", ble_command,
     "BLE link security. Usage: ble | ble pair | ble forget"},
    {"comm", comm_command, "comm_iface ports. Usage: comm [trace|clear]"},
    {"vif", vif_command, "Show vehicle interface owners and claims"},
    {"link", link_command,
     "Which transport carries the data link. Usage: link [drop|usb|ble]"},
    {"mode", mode_command,
     "Show or switch the link's grammar. Usage: mode [<name>]"},
    {"ctrl", ctrl_command, "Run a control command. Usage: ctrl <command>"},
    {NULL, NULL, NULL} // End of list marker
};

// Now we can define help_command as it needs the `commands` table
static void help_command(int argc, char **argv) {
    printf("Available commands:\n");
    for (int i = 0; commands[i].name != NULL; i++) {
        printf("  %-10s - %s\n", commands[i].name, commands[i].help);
    }
}

static void execute_command(char *line) {
    int argc = 0;
    char *argv[MAX_ARGS];
    char *token;

    // Trim leading/trailing whitespace
    char *start = line;
    while (isspace((unsigned char)*start))
        start++;
    char *end = line + strlen(line) - 1;
    while (end > start && isspace((unsigned char)*end))
        end--;
    *(end + 1) = '\0';

    if (strlen(start) == 0) {
        return; // Empty line
    }

    // Tokenize the command string
    token = strtok(start, " ");
    while (token != NULL && argc < MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    if (argc == 0) {
        return; // No command entered
    }

    // Find and execute the command
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(argv[0], commands[i].name) == 0) {
            commands[i].func(argc, argv);
            return;
        }
    }

    printf("Error: Unknown command '%s'. Type 'help' for a list of commands.\n",
           argv[0]);
}

/** @brief Returned by shell_getline() for a line that would not fit. */
#define SHELL_LINE_TOO_LONG (-1)

int shell_getline(char *line, size_t len) {
    int pos = 0;
    bool overflowed = false;

    if (!len) {
        return 0;
    }

    for (;;) {
        // Read one character at a time
        int c = getchar();

        if (c == EOF) {
            /*
             * Nothing waiting. The console VFS returns -1/EWOULDBLOCK rather
             * than blocking, and newlib latches that on the stream: without
             * the clearerr() every later getchar() returns EOF straight back
             * without ever calling the driver again, and the shell goes deaf
             * after the first idle poll.
             */
            clearerr(stdin);

            /* The console has no session task, so its idle poll is where its
             * own claims get released when recovery is asked for. */
            vif_shell_service();

            vTaskDelay(pdMS_TO_TICKS(SHELL_POLL_MS));
            continue;
        }

        // Handle newline characters (CR or LF)
        if (c == '\r' || c == '\n') {
            printf("\r\n");   // Echo newline
            fflush(stdout);   // before a command that prompts on its own
            line[pos] = '\0'; // Null-terminate the string

            /* Refuse the whole line rather than run a shortened one. This
             * console puts bytes on a vehicle bus: a request silently cut in
             * half is transmitted, answered with a timeout, and looks exactly
             * like a module that stopped talking. */
            return overflowed ? SHELL_LINE_TOO_LONG : pos;
        }
        // Handle backspace/delete
        else if (c == '\b' || c == 127) {
            if (pos > 0) {
                pos--;
                // Erase character from terminal (backspace, space, backspace)
                printf("\b \b");
                fflush(stdout);
            }
        }
        // Handle regular characters
        else if (isprint(c)) {
            if (pos < (int)len - 1) {
                line[pos++] = c;
                // Echo the character back to the terminal
                putc(c, stdout);
                fflush(stdout);
            } else {
                overflowed = true;
            }
        }
    }
}

void shell_loop(void) {
    char cmd_line[MAX_COMMAND_LEN];

    /* This task is the one allowed inside the drivers the shell brings up. */
    vif_shell_bind();

    printf("\n==================================\n");
    printf(" OpenDiag Command Shell\n");
    printf("==================================\n");
    printf("Type 'help' for a list of commands.\n");
    printf(SHELL_PROMPT);
    fflush(stdout);

    for (;;) {
        int n = shell_getline(cmd_line, sizeof(cmd_line));

        if (n == SHELL_LINE_TOO_LONG) {
            printf("line too long: %d characters at most\n",
                   (int)sizeof(cmd_line) - 1);
        } else if (n > 0) {
            execute_command(cmd_line);
        }

        printf(SHELL_PROMPT);
        fflush(stdout);
    }
}
