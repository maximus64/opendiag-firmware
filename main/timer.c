/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "timer.h"

/**
 * @brief Initializes a timer structure.
 * @param timer Pointer to the Timer structure.
 */
void Timer_init(Timer* timer) {
    timer->start_ms = 0;
    timer->timeout_ms = 0;
}

/**
 * @brief Starts the timer with a specified timeout.
 *
 * @param timer Pointer to the Timer structure.
 * @param timeout_ms The timeout duration in milliseconds.
 */
void Timer_start(Timer* timer, unsigned long timeout_ms) {
    timer->timeout_ms = timeout_ms;
    
    timer->start_ms = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
}

/**
 * @brief Checks if the timer has expired.
 *
 * @param timer Pointer to the Timer structure.
 * @return 1 if the timer has expired, 0 otherwise.
 */
int Timer_is_expired(Timer* timer) {
    unsigned long current_ms;
    
    current_ms = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;

    return (current_ms - timer->start_ms) >= timer->timeout_ms;
}

/**
 * @brief Gets the remaining time in milliseconds.
 *
 * @param timer Pointer to the Timer structure.
 * @return The remaining time in milliseconds. Returns 0 if expired.
 */
unsigned long Timer_remaining_ms(Timer* timer) {
    unsigned long current_ms, elapsed_ms;

    current_ms = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;

    elapsed_ms = current_ms - timer->start_ms;

    if (elapsed_ms >= timer->timeout_ms) {
        return 0;
    }

    return timer->timeout_ms - elapsed_ms;
}
