/**
 * @file gpio.h
 * @brief GPIO utility functions for toggling pins on Nordic nRF boards.
 *
 * Provides simple functions to manipulate GPIO pins, such as toggling
 * the state of a pin. This module uses the nRF SDK GPIO library.
 *
 * Author : Wisal Muhammad
 * Date   : 19/11/2025
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "stdint.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"


/**
 * @brief Toggle the state of a GPIO pin
 *
 * This function changes the logic level of the specified GPIO pin
 * from high to low or low to high.
 *
 * @param[in] pin_number Pointer to the pin number to toggle
 */
void gpio_toggle(uint8_t *pin_number);


#endif