/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_log.h"
#include "esp_system.h"
#include "utility.h"
#include "comm_iface.h"
#include "elm327_at.h"
#include "links.h"
#include "shell.h"
#include "slcan.h"
#include "vif.h"
#include "vif_ctrl.h"

#define TAG "SHELL"

// --- Configuration ---
#define SHELL_TASK_PRIORITY     5
#define SHELL_TASK_STACK_SIZE   4096
#define SHELL_PROMPT            "> "
#define MAX_COMMAND_LEN         128

/* How long to wait before looking for a keypress again. Short enough that
 * typing does not feel laggy, long enough not to spin a core. */
#define SHELL_POLL_MS           10
#define MAX_ARGS                10

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
 * Prints the current free heap size in bytes. Useful for monitoring memory usage.
 *
 * @param argc Argument count (unused)
 *param argv Argument vector (unused)
 */
static void heap_command(int argc, char **argv) {
    printf("Free Heap Size: %lu bytes\n", (unsigned long)esp_get_free_heap_size());
}

/**
 * @brief Command handler for 'tasks'
 *
 * Displays a table of all running FreeRTOS tasks, their states, priorities,
 * stack high water mark, and task numbers. This is an invaluable debugging tool.
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

/**
 * @brief The shell's own vif session.
 *
 * The console is not a comm_iface service - it talks to UART0 directly - but
 * it drives the same hardware as the protocol front-ends, so it holds its
 * claims the same way. A pin raised here is refused to ELM327, and vice versa,
 * instead of the two silently overwriting each other.
 */
static vif_session_t *shell_session;

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
        err = vif_pin_set(shell_session, pin, VIF_PIN_VOLTAGE, millivolt);
    }
    else {
        err = vif_pin_set(shell_session, pin, VIF_PIN_OFF, 0);
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

    err = vif_pin_set(shell_session, pin, state ? VIF_PIN_GROUND : VIF_PIN_OFF, 0);
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
    vif_pin_release_all(shell_session);
    printf("OK\n");
}

/**
 * @brief Command handler for 'boardid'
 *
 * Display the board id.
 */
static void boardid_command(int argc, char **argv) {
    uint8_t val = vif_board_id();
    printf("Board ID: %u\n", val);
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
    esp_err_t err = vif_calibrate_hs(shell_session);

    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
    }
}

static void vbattcal_command(int argc, char **argv) {
    esp_err_t err = vif_calibrate_vbatt(shell_session);

    if (err != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(err));
    }
}

/**
 * @brief Open a bus for a test command, unless this session already has it.
 */
static bool shell_bus_ready(vif_bus_t bus, const vif_bus_cfg_t *cfg)
{
    esp_err_t err;

    if (vif_bus_current(shell_session) == bus) {
        return true;
    }

    err = vif_bus_open(shell_session, bus, cfg);
    if (err != ESP_OK) {
        printf("ERROR: cannot open the bus: %s\n", esp_err_to_name(err));
        return false;
    }

    return true;
}

/** Replies one bus test command will print before handing the console back. */
#define SHELL_BUS_MAX_REPLIES 16

/**
 * @brief Send a hex string on a byte bus and print whatever answers.
 *
 * Shared by the kline, j1850 and vpw commands, which differ only in which bus
 * they ask vif for. The reply count is capped as well as the idle timeout: a
 * J1850 VPW bus carries periodic broadcasts, and waiting for a gap in those
 * would keep the console for as long as the vehicle is awake.
 */
static void shell_bytebus_command(vif_bus_t bus, const char *name,
                                  int argc, char **argv)
{
    uint8_t data[32];
    int rc;

    if (argc != 2) {
        printf("Usage: %s <hex string>|stop\n", name);
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        vif_bus_close(shell_session);
        printf("OK\n");
        return;
    }

    size_t len = strlen(argv[1]);
    if (len > sizeof(data) * 2) {
        printf("hex string too long\n");
        return;
    }

    rc = hex_string_to_u8_array(argv[1], len, data, sizeof(data));
    if (rc <= 0) {
        printf("Fail to parse the hex data: %s rc = %d\n", argv[1], rc);
        return;
    }

    if (!shell_bus_ready(bus, NULL)) {
        return;
    }

    rc = vif_raw_send(shell_session, data, rc);
    if (rc != 0) {
        printf("%s send failed: %d\n", name, rc);
        return;
    }

    for (int i = 0; i < SHELL_BUS_MAX_REPLIES; i++) {
        int ret = vif_raw_recv(shell_session, data, sizeof(data), pdMS_TO_TICKS(1000));

        if (ret > 0) {
            ESP_LOGI(TAG, "Got frame len %d", ret);
            ESP_LOG_BUFFER_HEX(TAG, data, ret);
        }
        else if (ret == -1) {
            ESP_LOGI(TAG, "%s receive timeout", name);
            break;
        }
        else {
            ESP_LOGE(TAG, "%s receive failed: %d", name, ret);
            break;
        }
    }
}

static void kline_command(int argc, char **argv) {
    shell_bytebus_command(VIF_BUS_KLINE, "kline", argc, argv);
}

static void j1850_command(int argc, char **argv) {
    shell_bytebus_command(VIF_BUS_J1850_PWM, "j1850", argc, argv);
}

static void j1850_vpw_command(int argc, char **argv) {
    shell_bytebus_command(VIF_BUS_J1850_VPW, "vpw", argc, argv);
}

/**
 * @brief Command handler for 'can'
 *
 * Brings the CAN driver up and down repeatedly, which is how the teardown path
 * gets exercised on hardware.
 */
static void can_command(int argc, char **argv) {
    vif_bus_cfg_t cfg = { .bitrate = 500000 };
    int cycles = 1000;

    if (argc > 1) {
        cycles = atoi(argv[1]);
    }

    for (int i = 0; i < cycles; i++) {
        printf("Starting CAN bus test... (%d/%d)\n", i + 1, cycles);
        if (vif_bus_open(shell_session, VIF_BUS_CAN, &cfg) != ESP_OK) {
            printf("ERROR: cannot open the CAN bus\n");
            return;
        }
        printf("Tearing down CAN bus...\n");
        vif_bus_close(shell_session);
    }
}

/**
 * @brief Command handler for 'vif'
 *
 * Prints the sessions, their front-ends and their claims. Switching a link's
 * grammar is 'mode', which goes through the same control plane a client uses.
 */
static void vif_command(int argc, char **argv) {
    vif_print_debug_info();
}

/**
 * @brief Command handler for 'mode'
 *
 * The USB half of the control plane. The shell has no data link of its own, so
 * unlike the BLE control characteristic it has to be told which link it means:
 *
 *   mode                  every link and the grammar it is running
 *   mode <link>           that link's grammar
 *   mode <link> <name>    switch it, keeping the session's claims
 *
 * The switch itself is vif_ctrl_exec(), so the shell and a BLE client cannot
 * disagree about what a mode change does.
 */
static void mode_command(int argc, char **argv) {
    char out[VIF_CTRL_REPLY_MAX];
    char cmd[VIF_CTRL_REPLY_MAX];
    vif_session_t *target;

    if (argc == 1) {
        const vif_frontend_t *fe;

        printf("Links:\n");
        for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
            vif_session_t *s = vif_session_at(i);
            const char *name = vif_session_name(s);

            /* Sessions that carry no protocol are not links and cannot be
             * switched. The shell holds one for its claims; "vif" shows it. */
            if (!name || !vif_session_is_link(s)) {
                continue;
            }

            fe = vif_session_default_frontend(s);
            printf("  %-8s %-8s (default %s)\n", name,
                   vif_session_frontend_name(s) ? vif_session_frontend_name(s) : "-",
                   fe ? fe->name : "-");
        }

        printf("Modes:");
        for (size_t i = 0; (fe = vif_frontend_at(i)) != NULL; i++) {
            printf(" %s", fe->name);
        }
        printf("\n");
        return;
    }

    if (argc > 3) {
        printf("Usage: mode [<link> [<name>]]\n");
        return;
    }

    target = vif_session_find(argv[1]);
    if (!target) {
        printf("ERROR: no link named '%s'\n", argv[1]);
        return;
    }

    if (argc == 2) {
        snprintf(cmd, sizeof(cmd), "MODE");
    }
    else {
        snprintf(cmd, sizeof(cmd), "MODE=%s", argv[2]);
    }

    vif_ctrl_exec(target, cmd, strlen(cmd), out, sizeof(out));
    printf("%s\n", out);
}

/**
 * @brief Command handler for 'ctrl'
 *
 * Runs a raw control command against a link - the same text a BLE client
 * writes to the control characteristic. Useful for trying one without a phone.
 */
static void ctrl_command(int argc, char **argv) {
    char out[VIF_CTRL_REPLY_MAX];
    vif_session_t *target;

    if (argc != 3) {
        printf("Usage: ctrl <link> <ID|MODE|MODE=name|BUS|RESET>\n");
        return;
    }

    target = vif_session_find(argv[1]);
    if (!target) {
        printf("ERROR: no link named '%s'\n", argv[1]);
        return;
    }

    vif_ctrl_exec(target, argv[2], strlen(argv[2]), out, sizeof(out));
    printf("%s\n", out);
}

// --- Command Table ---
// This table registers all available commands.
// The list MUST be terminated with a {NULL, NULL, NULL} entry.
/**
 * @brief Command handler for 'comm'
 *
 * Shows the registered comm_iface ports and their state.
 */
static void comm_command(int argc, char **argv) {
    comm_print_debug_info();
}

const shell_command_t commands[] = {
    { "help",     help_command,     "Show this help message" },
    { "heap",     heap_command,     "Show free heap size" },
    { "tasks",    tasks_command,    "Show list of running tasks" },
    { "reboot",   reboot_command,   "Reboot the device" },
    { "hsset",    hsset_command,    "Drive a high side pin. Usage: hsset <pin> <millivolt>" },
    { "lsset",    lsset_command,    "Ground the low side pin. Usage: lsset <pin> <state>" },
    { "pinoff",   pinoff_command,   "Release every pin this console holds" },
    { "boardid",  boardid_command,  "Show board id value" },
    { "vbatt",    vbatt_command,    "Show Battery Voltage" },
    { "hsvsense", hsvsense_command, "Show high side voltage sense" },
    { "hscal",    hscal_command,    "HS voltage calibration"},
    { "vbattcal", vbattcal_command, "VBatt voltage calibration"},
    { "kline",    kline_command,    "K-Line test. Usage: kline <hex string>|stop"},
    { "j1850",    j1850_command,    "J1850 PWM test. Usage: j1850 <hex string>|stop"},
    { "vpw",      j1850_vpw_command, "J1850 VPW test. Usage: vpw <hex string>|stop"},
    { "can",      can_command,      "CAN bring up/tear down loop. Usage: can [cycles]"},
    { "comm",     comm_command,     "Show comm_iface ports and their state"},
    { "vif",      vif_command,      "Show vehicle interface sessions and claims"},
    { "mode",     mode_command,     "Show or switch a link's grammar. Usage: mode [<link> [<name>]]"},
    { "ctrl",     ctrl_command,     "Run a control command. Usage: ctrl <link> <command>"},
    { NULL,       NULL,             NULL } // End of list marker
};

// Now we can define help_command as it needs the `commands` table
static void help_command(int argc, char **argv) {
    printf("Available commands:\n");
    for (int i = 0; commands[i].name != NULL; i++) {
        printf("  %-10s - %s\n", commands[i].name, commands[i].help);
    }
}


// --- Core Shell Logic ---

/**
 * @brief Parses a command line string and executes the corresponding command.
 *
 * @param line The raw command line string to parse.
 */
static void execute_command(char *line) {
    int argc = 0;
    char *argv[MAX_ARGS];
    char *token;

    // Trim leading/trailing whitespace
    char *start = line;
    while (isspace((unsigned char)*start)) start++;
    char *end = line + strlen(line) - 1;
    while (end > start && isspace((unsigned char)*end)) end--;
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

    printf("Error: Unknown command '%s'. Type 'help' for a list of commands.\n", argv[0]);
}

int shell_getline(char *line, size_t len)
{
    int pos = 0;

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
            vif_session_service(shell_session);

            vTaskDelay(pdMS_TO_TICKS(SHELL_POLL_MS));
            continue;
        }

        // Handle newline characters (CR or LF)
        if (c == '\r' || c == '\n') {
            printf("\r\n"); // Echo newline
            fflush(stdout);   // before a command that prompts on its own
            line[pos] = '\0'; // Null-terminate the string
            return pos;
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
        else if (isprint(c) && pos < len - 1) {
            line[pos++] = c;
            // Echo the character back to the terminal
            putc(c, stdout);
            fflush(stdout);
        }
    }
}

void shell_loop(void)
{
    char cmd_line[MAX_COMMAND_LEN];

    shell_session = vif_session_open(LINK_NAME_SHELL, COMM_INVALID_PORT_ID);
    if (!shell_session) {
        printf("WARNING: no vif session for the console; hardware commands "
               "will be refused\n");
    }

    printf("\n==================================\n");
    printf(" OpenDiag Command Shell\n");
    printf("==================================\n");
    printf("Type 'help' for a list of commands.\n");
    printf(SHELL_PROMPT);
    fflush(stdout); 

    for (;;) {
        int n = shell_getline(cmd_line, sizeof(cmd_line));

        if (n) {
            execute_command(cmd_line);
        }

        printf(SHELL_PROMPT);
        fflush(stdout);
    }
}
