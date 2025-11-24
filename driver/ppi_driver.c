/**
 * @file ppi_driver.c
 * @brief PPI (Programmable Peripheral Interconnect) driver implementation.
 *
 * Provides functions to configure and manage PPI channels that connect
 * hardware events and tasks without CPU intervention. Supports:
 *  - SAADC sampling triggered by TIMER events
 *  - TWIM/I2C transfers triggered by TIMER events
 *  - Safe uninitialization of PPI channels
 *
 * Author : Wisal Muhammad
 * Date   : 19/11/2025
 */

#include "ppi_driver.h"


// <========================= PPI ===================================>

#if APP_ENABLE_SAADC
/**
 * @brief PPI channel used for TIMER -> SAADC linkage
 */
static nrf_ppi_channel_t ppi_channel_saadc;
#endif
#if APP_ENABLE_TWIM
/**
 * @brief PPI channel used for TIMER -> TWIM/I2C linkage
 */
static nrf_ppi_channel_t ppi_channel_i2c;
#endif


#if APP_ENABLE_SAADC

/**
 * @brief Initialize PPI channel to trigger SAADC sampling from a timer
 *
 * Allocates a PPI channel, links TIMER COMPARE0 event to the SAADC SAMPLE task,
 * and enables the channel.
 *
 * @param[in] my_timer Pointer to the timer instance used as the event source
 */
void ppi_saadc_init(nrfx_timer_t const *my_timer) {

  ret_code_t err_code;
  uint32_t timer_evt_addr  = nrfx_timer_event_address_get(my_timer, NRF_TIMER_EVENT_COMPARE0);
  uint32_t saadc_task_addr = nrfx_saadc_sample_task_get();
  
  // Allocate channel
  err_code = nrfx_ppi_channel_alloc(&ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  // Assign event -> task
  err_code = nrfx_ppi_channel_assign(ppi_channel_saadc, timer_evt_addr, saadc_task_addr);
  APP_ERROR_CHECK(err_code);
  // Enable channel
  err_code = nrfx_ppi_channel_enable(ppi_channel_saadc);
  APP_ERROR_CHECK(err_code); 
}

#endif


#if APP_ENABLE_TWIM

/**
 * @brief Initialize PPI channel to trigger TWIM/I2C transfers from a timer
 *
 * Allocates a PPI channel, links TIMER COMPARE0 event to the TWIM START task,
 * and enables the channel.
 *
 * @param[in] my_timer Pointer to the timer instance used as the event source
 * @param[in] i2c      Pointer to the TWIM/I2C instance to be triggered
 */
void ppi_twim_init(nrfx_timer_t const *my_timer,nrfx_twim_t const *i2c) {
  ret_code_t err_code;
  uint32_t timer_evt_addr  = nrfx_timer_event_address_get(my_timer, NRF_TIMER_EVENT_COMPARE0);
 
  err_code = nrfx_ppi_channel_alloc(&ppi_channel_i2c);
  APP_ERROR_CHECK(err_code);

  uint32_t twim_task_addr = nrfx_twim_start_task_get(i2c, NRFX_TWIM_XFER_TXRX);
  err_code = nrfx_ppi_channel_assign(ppi_channel_i2c, timer_evt_addr, twim_task_addr);
  APP_ERROR_CHECK(err_code);

  // Enable channel
  
  err_code = nrfx_ppi_channel_enable(ppi_channel_i2c);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("PPI channel linked: TIMER -> SAADC, TIMER -> TWIM(I2C)");
}

#endif


/**
 * @brief Uninitialize all configured PPI channels
 *
 * Disables and frees the PPI channels safely. Handles both SAADC and TWIM
 * channels depending on the application configuration.
 */
void ppi_uninit(void) {
  // Disable first (safe)
  ret_code_t err;
  #if APP_ENABLE_SAADC
  err = nrfx_ppi_channel_disable(ppi_channel_saadc);
  if (err != NRFX_SUCCESS && err != NRFX_ERROR_INVALID_STATE) {
    APP_ERROR_CHECK(err);
  }

  err = nrfx_ppi_channel_free(ppi_channel_saadc);
  if (err != NRFX_SUCCESS && err != NRFX_ERROR_INVALID_PARAM) {
    APP_ERROR_CHECK(err);
  }
  #endif

  #if APP_ENABLE_TWIM
  err = nrfx_ppi_channel_disable(ppi_channel_i2c);
  if (err != NRFX_SUCCESS && err != NRFX_ERROR_INVALID_STATE) {
    APP_ERROR_CHECK(err);
  }

  err = nrfx_ppi_channel_free(ppi_channel_i2c);
  if (err != NRFX_SUCCESS && err != NRFX_ERROR_INVALID_PARAM) {
    APP_ERROR_CHECK(err);
  }
  #endif

  NRF_LOG_INFO("PPI channel released");
}