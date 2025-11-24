/*
* Author : Wisal Muhammad
* Date : 19/11/2025
*
* @file ppi_driver.h
 * @brief PPI (Programmable Peripheral Interconnect) driver for nRF peripherals.
 *
 * This header provides function prototypes to configure and manage PPI channels
 * to connect hardware events and tasks between peripherals without CPU intervention.
 *
 * Features:
 *  - Connects TIMER events to SAADC sampling (APP_ENABLE_SAADC)
 *  - Connects TIMER events to TWIM/I2C transfers (APP_ENABLE_TWIM)
 *  - Provides uninitialization function to free PPI resources
*/



#ifndef PPI_DRIVER_H__
#define PPI_DRIVER_H__

#include "nrfx_ppi.h"
#include "nrfx_timer.h"
#include "nrfx_saadc.h"
#include "nrfx_twim.h"
#include "nrf_log.h"

#if APP_ENABLE_SAADC
/**
 * @brief Initialize PPI for SAADC sampling triggered by a timer
 *
 * This function sets up PPI channels to trigger SAADC conversions
 * automatically when the specified timer generates events.
 *
 * @param[in] my_timer Pointer to the timer instance to use as trigger
 */
void ppi_saadc_init(nrfx_timer_t const *my_timer);
#endif

#if APP_ENABLE_TWIM
/**
 * @brief Initialize PPI for TWIM/I2C transfers triggered by a timer
 *
 * This function configures PPI channels to trigger I2C transfers
 * automatically when the specified timer generates events.
 *
 * @param[in] my_timer Pointer to the timer instance to use as trigger
 * @param[in] i2c      Pointer to the TWIM/I2C instance to be triggered
 */
void ppi_twim_init(nrfx_timer_t const *my_timer,nrfx_twim_t const *i2c);
#endif

/**
 * @brief Uninitialize PPI channels
 *
 * Frees all configured PPI channels and disables them.
 */
void ppi_uninit(void);

#endif