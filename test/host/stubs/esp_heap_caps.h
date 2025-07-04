/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_heap_caps.h
 * @brief Host stand-in for the ESP-IDF capability-aware allocator.
 *
 * The firmware only names a capability when creating a ring buffer, and the
 * host has one kind of memory, so the flag is inert.
 */

#pragma once

#define MALLOC_CAP_DEFAULT (1 << 12)
