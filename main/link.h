/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file link.h
 * @brief Which transport carries the one data link.
 *
 * A user talks to this adapter over USB or over BLE, never both, so there is
 * one session and it follows whichever transport has a client. First
 * connected wins.
 *
 * That rule needs an override, because a developer's host holds CDC0 open for
 * the whole session: the link binds to USB at boot and BLE can never take it,
 * so a phone connects, writes, and is met with silence. "link ble" hands it
 * over; "link drop" lets go so the next transport to connect can have it.
 */

#include <stdbool.h>

typedef enum {
    LINK_NONE = 0,
    LINK_USB,
    LINK_BLE,
} link_transport_t;

/** @brief Which transport holds the link, or LINK_NONE. */
link_transport_t link_holder(void);

/**
 * @brief Hand the link to @p t. LINK_NONE lets go of it.
 *
 * Whoever held it is cut off the way a departing client is - grammar back to
 * its default, claims released - because the next one must not inherit a
 * conversation it did not start. Letting go with nothing named offers the
 * link to any other transport that already has a client.
 */
void link_set(link_transport_t t);

/** @brief Name for the shell and logs. */
const char *link_transport_name(link_transport_t t);

/** @brief True while that transport has a client attached. */
bool link_transport_connected(link_transport_t t);
