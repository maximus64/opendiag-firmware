/* SPDX-License-Identifier: GPL-3.0-only */
#include <ctype.h>
#include <stdio.h>
#include "esp_timer.h"

void hexdump(const void *data, size_t size) {
    const unsigned char *byte_data = (const unsigned char *)data;
    size_t i = 0;

    while (i < size) {
        char ascii[17] = {0};

        for (size_t j = 0; j < 16; ++j) {
            if (j == 8) {
                printf(" ");
            }

            if (i + j < size) {
                printf("%02X ", byte_data[i + j]);
                ascii[j] = isprint(byte_data[i + j]) ? byte_data[i + j] : '.';
            } else {
                // Pad the hex output for alignment.
                printf("   ");
            }
        }

        printf("|  %s\n", ascii);
        i += 16;
    }
}

int8_t hex_char_to_int(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

int hex_string_to_u8_array(const char *hex_str, size_t len,
                           uint8_t *output_array, size_t output_size) {
    size_t i = 0;
    int byte_count = 0;
    int8_t high_nibble, low_nibble;

    if ((len / 2) > output_size) {
        return -2;
    }

    while (len >= 2) {
        high_nibble = hex_char_to_int(hex_str[i]);
        if (high_nibble < 0) {
            return -1;
        }

        low_nibble = hex_char_to_int(hex_str[i + 1]);
        if (low_nibble < 0) {
            return -1;
        }

        output_array[byte_count] = (uint8_t)((high_nibble << 4) | low_nibble);

        i += 2;
        len -= 2;
        byte_count++;
    }

    return byte_count;
}

int hex_string_to_u32_be(const char *hex_str, size_t len, uint32_t *output) {
    uint32_t val = 0;
    size_t i = 0;

    if (len > 8) {
        return -1;
    }

    while (len--) {
        int8_t nibble = hex_char_to_int(hex_str[i++]);
        if (nibble < 0) {
            return -1;
        }

        val = (val << 4) | nibble;
    }

    *output = val;
    return 0;
}

void delay_us(uint32_t us) {
    int64_t start_time = esp_timer_get_time();
    while (esp_timer_get_time() - start_time < us) {
        continue;
    }
}
