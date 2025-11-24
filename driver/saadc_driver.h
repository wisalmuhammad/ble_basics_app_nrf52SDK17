/*
* Author : Wisal Muhammad
* Date : 19/11/2025
*/

/*
 * @file saadc.h
 *
 * Header file for SAADC (Successive Approximation ADC) driver wrapper.
 * This module provides a simplified interface for initializing the SAADC,
 * receiving converted voltage samples through a callback, and uninitializing
 * the peripheral when no longer needed.
 */

#ifndef _SAADC_H
#define _SAADC_H

#include "app_config.h"

/*
 * Optional inclusion based on application configuration.
 * If APP_ENABLE_SAADC is disabled, this module is excluded from compilation.
 */
#if APP_ENABLE_SAADC

#include "stdint.h"

/*
 * @brief SAADC sample callback function type.
 *
 * The SAADC driver converts raw ADC readings into a floating-point voltage.
 * The application provides a callback that receives this voltage value
 * whenever a new sample becomes available.
 *
 * @param value  Converted voltage in volts.
 */
typedef void (*saadc_sample_cb_t)(float value);

/*
 * @brief Initialize the SAADC peripheral.
 *
 * Sets up the SAADC channel, configures buffers, registers the callback,
 * and prepares the peripheral for sampling. Sampling behavior (continuous or
 * on-demand) depends on the implementation inside saadc.c.
 *
 * @param callback Function to be invoked for every completed sample.
 */
void saadc_init(saadc_sample_cb_t callback);

/*
 * @brief Uninitialize the SAADC peripheral.
 *
 * Stops sampling, disables the SAADC, and releases allocated buffers.
 * Call this function when SAADC functionality is no longer required.
 */
void saadc_uinit(void);

#endif // APP_ENABLE_SAADC

#endif // _SAADC_H
