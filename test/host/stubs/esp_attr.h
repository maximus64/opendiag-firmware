/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_attr.h
 * @brief Host stand-in for the ESP-IDF placement attributes.
 *
 * On target IRAM_ATTR keeps a function off the flash cache so an interrupt
 * can call it with predictable timing. On the host there is no flash and no
 * cache, so the attributes are simply absent - the code they annotate is
 * identical either way, which is the point.
 */

#pragma once

#define IRAM_ATTR
#define DRAM_ATTR
