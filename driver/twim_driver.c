/*
* Author : Wisal Muhammad
* Date : 19/11/2025
*/
/**
 * @file twim_driver.c
 * @brief Simple TWIM (I2C) driver wrapper using nrfx_twim.
 *
 * This module provides initialization, uninitialization, and
 * read/write operations for I2C (TWIM) peripheral.
 *
 * Features:
 *  - Asynchronous transfer with completion flag
 *  - Callback support for TWIM events
 *  - Supports TX and TX+RX transfers
 *
 */

#include "twim_driver.h"

#if APP_ENABLE_TWIM

/**
 * @brief User-provided callback for I2C events
 */
static i2c_evt_cb_t callback;

/**
 * @brief Transfer completion flag
 */
static volatile bool m_xfer_done = false;

/**
 * @brief TWIM instance
 */
const nrfx_twim_t i2c = NRFX_TWIM_INSTANCE(NRFX_TWIM_IDX);

/**
 * @brief Checks if the last I2C transfer is complete
 *
 * @return true if transfer finished, false otherwise
 */
bool i2c_transfer_done(void) {
    return m_xfer_done;
}


/**
 * @brief TWIM event handler
 *
 * Handles TWIM events like transfer complete, address NACK, data NACK.
 * Invokes the user-provided callback if assigned.
 *
 * @param[in] p_event   Pointer to TWIM event
 * @param[in] p_context User-defined context (unused)
 */
static void twim_event_handler(nrfx_twim_evt_t const *p_event, void *p_context) {
  NRF_LOG_ERROR("twim_event_handler evt type: %d", (*p_event).type);
  if (p_event->type == NRFX_TWIM_EVT_DONE) {
    // Transfer complete
    m_xfer_done = true;
  } else if (p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK) {
    // Address NACK
    NRF_LOG_INFO("Address NACK");
  } else if (p_event->type == NRFX_TWIM_EVT_DATA_NACK) {
    // Data NACK
    NRF_LOG_INFO("Data NACK");
  }

  if (callback)
    callback(p_event);
}


/**
 * @brief Initialize TWIM (I2C) peripheral
 *
 * Configures SCL and SDA pins, sets frequency, priority, and assigns callback.
 *
 * @param[in] scl_pin  Pin number for SCL
 * @param[in] sda_pin  Pin number for SDA
 * @param[in] call_back User callback for TWIM events
 */
void i2c_init(uint8_t scl_pin, uint8_t sda_pin, i2c_evt_cb_t call_back) {
  callback = call_back;
  ret_code_t err_code;
  nrfx_twim_config_t config = {.scl                = scl_pin,
                               .sda                = sda_pin,
                               .frequency          = NRF_TWIM_FREQ_400K,    // 400 kHz
                               .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
                               .hold_bus_uninit    = false};
  m_xfer_done               = false;
  ret_code_t err            = nrfx_twim_init(&i2c, &config, twim_event_handler, NULL);
  APP_ERROR_CHECK(err);

  nrfx_twim_enable(&i2c);
}

/**
 * @brief Uninitialize TWIM peripheral
 *
 * Disables TWIM instance and resets transfer completion flag.
 */
void i2c_uninit(void) {
  m_xfer_done = false;
  nrfx_twim_uninit(&i2c);
}

/**
 * @brief Transmit data over I2C
 *
 * @param[in] address  I2C slave address
 * @param[in] p_data   Pointer to data buffer to send
 * @param[in] length   Number of bytes to send
 * @return NRF_SUCCESS on success, error code otherwise
 */
ret_code_t i2c_tx(uint8_t address, uint8_t *p_data, size_t length) {
  m_xfer_done = false;
  ret_code_t err_code;
  nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(address, p_data, length);
  err_code                   = nrfx_twim_xfer(&i2c, &desc, 0);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_DEBUG("Error, unable to trasnfer");
  }

  return err_code;
}

/**
 * @brief Perform combined TX+RX I2C transfer
 *
 * @param[in] addr   I2C slave address
 * @param[in] p_tx   Pointer to transmit buffer
 * @param[in] tx_len Length of transmit buffer
 * @param[in] p_rx   Pointer to receive buffer
 * @param[in] rx_len Length of receive buffer
 * @param[in] flags  Transfer flags (0 for default)
 * @return NRF_SUCCESS on success, error code otherwise
 */
ret_code_t i2c_tx_rx(uint8_t addr, uint8_t *p_tx, size_t tx_len, uint8_t *p_rx, size_t rx_len, uint32_t flags) {
  m_xfer_done = false;
  ret_code_t err_code;

  nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TXRX(addr, p_tx, tx_len, p_rx, rx_len);
  err_code                   = nrfx_twim_xfer(&i2c, &desc, flags);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_DEBUG("Error while transfer and read data");
  }

  return err_code;
}


#endif