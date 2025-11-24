/*
* Author : Wisal Muhammad
* Date : 19/11/2025
*/
/*
* @file twim_driver.h
 * @brief Header file for TWIM (I2C) driver using nrfx_twim.
 *
 * Provides function prototypes, type definitions, and constants for
 * initializing, uninitializing, transmitting, and receiving data
 * over the I2C bus using the Nordic TWIM peripheral.
*/

#ifndef TWIM_H_
#define TWIM_H_

#include "app_config.h"
#include "nrfx_twim.h"
#include "nrf_log.h"

#if APP_ENABLE_TWIM

/**
 * @brief TWIM instance index to use
 */
#define NRFX_TWIM_IDX 0    ///< Using TWIM0 instance

/**
 * @brief TWIM instance handle
 */
extern const nrfx_twim_t i2c;

/**
 * @brief User-defined I2C event callback
 *
 * This callback is invoked by the TWIM driver on events such as
 * transfer complete, address NACK, or data NACK.
 *
 * @param[in] p_event Pointer to TWIM event structure
 */
typedef void (*i2c_evt_cb_t)(nrfx_twim_evt_t const *p_event);

/**
 * @brief Initialize TWIM (I2C) peripheral
 *
 * Configures the TWIM peripheral with the specified SCL and SDA pins,
 * sets the frequency, interrupt priority, and registers the event callback.
 *
 * @param[in] scl_pin   Pin number for SCL
 * @param[in] sda_pin   Pin number for SDA
 * @param[in] call_back User-defined callback for TWIM events
 */
void i2c_init(uint8_t scl_pin, uint8_t sda_pin, i2c_evt_cb_t call_back);

/**
 * @brief Uninitialize TWIM peripheral
 *
 * Disables the TWIM instance and frees resources.
 */
void i2c_uninit(void);

/**
 * @brief Transmit data over I2C
 *
 * @param[in] address I2C slave address
 * @param[in] p_data  Pointer to data buffer to transmit
 * @param[in] length  Number of bytes to transmit
 * @return NRF_SUCCESS on success, otherwise error code
 */
ret_code_t i2c_tx(uint8_t address, uint8_t *p_data, size_t length);

/**
 * @brief Perform combined I2C transmit and receive
 *
 * @param[in] addr    I2C slave address
 * @param[in] p_tx    Pointer to transmit buffer
 * @param[in] tx_len  Length of transmit buffer
 * @param[in] p_rx    Pointer to receive buffer
 * @param[in] rx_len  Length of receive buffer
 * @param[in] flags   Transfer flags (0 for default)
 * @return NRF_SUCCESS on success, otherwise error code
 */
ret_code_t i2c_tx_rx(uint8_t addr, uint8_t *p_tx, size_t tx_len, uint8_t *p_rx, size_t rx_len, uint32_t flags);

/**
 * @brief Check if the last I2C transfer is complete
 *
 * @return true if transfer finished, false otherwise
 */
bool i2c_transfer_done(void);

#endif

#endif