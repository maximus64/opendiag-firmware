/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @brief Dumps a block of memory to stdout in a standard hexdump format.
 *
 * The output is formatted with 16 bytes per line, showing both the hexadecimal
 * and printable ASCII characters. Non-printable characters are shown as '.'.
 *
 * @param data A const pointer to the memory region to be dumped.
 * @param size The total number of bytes to dump.
 */
void hexdump(const void *data, size_t size);

/**
 * @brief Converts a hex string into a byte array.
 *
 * @param hex_str     Input hex characters (not required to be null terminated).
 * @param len         Number of characters of hex_str to convert.
 * @param output_array Destination buffer.
 * @param output_size Capacity of output_array in bytes.
 * @return Number of bytes written, -1 on an invalid hex character,
 *         or -2 if output_array is too small to hold len/2 bytes.
 */
int hex_string_to_u8_array(const char *hex_str, size_t len,
                           uint8_t *output_array, size_t output_size);
int hex_string_to_u32_be(const char *hex_str, size_t len, uint32_t *output);
int8_t hex_char_to_int(char c);
void delay_us(uint32_t us);
