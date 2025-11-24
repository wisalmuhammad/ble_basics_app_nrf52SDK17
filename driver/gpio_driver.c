/**
 * @file gpio_driver.c
 * @brief GPIO driver implementation for toggling pins on Nordic nRF boards.
 *
 * This module provides functions to manipulate GPIO pins, including toggling
 * their state. It uses the nRF SDK GPIO library and provides logging for debugging.
 *
 * Author : Wisal Muhammad
 * Date   : 19/11/2025
 */

#include "gpio_driver.h"

/**
 * @brief Toggle the state of a GPIO pin
 *
 * This function toggles the logic level of a pin specified by `pin_number`.
 * If the pin number matches 0–3, it maps to the corresponding board LED:
 *  - 0 → BSP_LED_0
 *  - 1 → BSP_LED_1
 *  - 2 → BSP_LED_2
 *  - 3 or higher → BSP_LED_3
 *
 * Logs an informational message indicating which LED is toggled.
 *
 * @param[in] pin_number Pointer to the pin number to toggle
 */
void gpio_toggle(uint8_t *pin_number) {
  NRF_LOG_INFO("LED %d should be toggle", *pin_number);

  nrf_gpio_pin_toggle(*pin_number == 0x00       ? BSP_LED_0
                          : *pin_number == 0x01 ? BSP_LED_1
                          : *pin_number == 0x02 ? BSP_LED_2
                                                : BSP_LED_3);
}