/*
* Author : Wisal Muhammad
* Date : 19/11/2025
*/
/**
 * @file saadc_driver.c
 * @brief Driver for Nordic SAADC (Successive Approximation ADC) to measure voltage on a single channel.
 *
 * This file provides initialization, callback handling, and uninitialization
 * functions for the SAADC peripheral using the nRF SDK.
 *
 * Features:
 *  - Single-ended ADC on AIN0
 *  - Uses internal 0.6 V reference
 *  - Gain 1/6
 *  - 12-bit resolution
 *  - Callback on voltage change
 *
 */

#include "app_config.h"
#include "saadc_driver.h"
#include "nrf_log.h"
#include "nrfx_saadc.h"

#if APP_ENABLE_SAADC

// <================================== SAADC ==============================>

/**
 * @brief ADC Gain (1/6)
 */
#define ADC_GAIN 6.0f

/**
 * @brief Internal reference voltage in volts
 */
#define VREF_INTERNAL 0.6f

/**
 * @brief Maximum ADC value for 12-bit resolution
 */
#define ADC_MAX_VALUE 4095

/**
 * @brief Callback function pointer for delivering sampled voltage
 */
static saadc_sample_cb_t call_back;

/**
 * @brief DMA buffer for SAADC
 */
static nrf_saadc_value_t m_buffer[1];

/**
 * @brief Stores the previous voltage value to avoid repeated callbacks
 */
static float previous_vol_value = 0.0f;


/**
 * @brief SAADC event handler
 *
 * This function is called by the SAADC driver when a conversion is complete.
 * It converts the raw ADC value to voltage and invokes the user callback
 * if the voltage has changed.
 *
 * @param[in] p_event Pointer to the SAADC event structure
 */

static void saadc_callback(nrfx_saadc_evt_t const *p_event) {
  if (p_event->type == NRFX_SAADC_EVT_DONE) {
    nrf_saadc_value_t value = p_event->data.done.p_buffer[0];
    // convert volatge

    // Avoid negative values in single-ended mode
    uint16_t adc_raw = (value < 0) ? 0 : (uint16_t)value;

    float voltage = ((float)value / ADC_MAX_VALUE) * (VREF_INTERNAL * ADC_GAIN);
    // NRF_LOG_INFO("ADC Value: %d, Voltage: " NRF_LOG_FLOAT_MARKER " V", adc_raw, NRF_LOG_FLOAT(voltage));
    if (voltage != previous_vol_value) {
      call_back(voltage);
    }
    previous_vol_value = voltage;
    // resue the same buffer for the next conversion

    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, 1));
  }

  // m_sampling_active = true;

  // if (m_sampling_active) {
  //   APP_ERROR_CHECK(nrfx_saadc_sample());
  // }
}


/**
 * @brief Initializes the SAADC driver and channel
 *
 * Configures SAADC peripheral with default settings and single-ended channel on AIN0.
 * Sets up internal reference, gain, and acquisition time.
 *
 * @param[in] callback User-defined callback function invoked on each voltage sample
 */

void saadc_init(saadc_sample_cb_t callback) {
  // Step SAADC with default CONFIG
  ret_code_t err_code;
  nrfx_saadc_config_t config = NRFX_SAADC_DEFAULT_CONFIG;
  config.interrupt_priority  = APP_IRQ_PRIORITY_LOWEST;
  err_code                   = nrfx_saadc_init(&config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  channel_config.gain                       = NRF_SAADC_GAIN1_6;
  channel_config.acq_time                   = NRF_SAADC_ACQTIME_10US;
  channel_config.reference                  = NRF_SAADC_REFERENCE_INTERNAL;

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  // Provide initial buffer before first sample
  err_code = nrfx_saadc_buffer_convert(m_buffer, 1);
  APP_ERROR_CHECK(err_code);

  call_back = callback;

  NRF_LOG_INFO("SAADC initialized");
}


/**
 * @brief Uninitializes the SAADC driver and channel
 *
 * Frees SAADC resources and uninitializes the channel and driver.
 */
void saadc_uinit(void) {
  // 4) Uninit channel & driver
  nrfx_saadc_channel_uninit(0);
  nrfx_saadc_uninit();

  // NRF_LOG_INFO("SAADC uninitialized");
}

#endif
