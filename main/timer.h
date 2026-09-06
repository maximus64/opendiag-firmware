/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @brief Timer structure to hold the start time and the timeout duration.
 */
typedef struct {
    unsigned long start_ms;
    unsigned long timeout_ms;
} Timer;

/**
 * @brief Initializes a timer structure.
 * @param timer Pointer to the Timer structure.
 */
void Timer_init(Timer *timer);

/**
 * @brief Starts the timer with a specified timeout.
 *
 * @param timer Pointer to the Timer structure.
 * @param timeout_ms The timeout duration in milliseconds.
 */
void Timer_start(Timer *timer, unsigned long timeout_ms);

/**
 * @brief Checks if the timer has expired.
 *
 * @param timer Pointer to the Timer structure.
 * @return 1 if the timer has expired, 0 otherwise.
 */
int Timer_is_expired(Timer *timer);

/**
 * @brief Milliseconds since the timer was started.
 *
 * Unlike Timer_remaining_ms(), this keeps counting past the timeout - so a
 * timer started only to measure something does not have to be given a
 * deadline it will never use.
 *
 * @param timer Pointer to the Timer structure.
 * @return Elapsed milliseconds.
 */
unsigned long Timer_elapsed_ms(Timer *timer);

/**
 * @brief Gets the remaining time in milliseconds.
 *
 * @param timer Pointer to the Timer structure.
 * @return The remaining time in milliseconds. Returns 0 if expired.
 */
unsigned long Timer_remaining_ms(Timer *timer);
