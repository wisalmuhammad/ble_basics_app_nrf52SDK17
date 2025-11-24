/*
@file custom_service.c
@brief Implementation of CustomService class for handling custom operations.

* Author : Wisal Muhammad
* Date : 19/11/2025

*/

#include "custom_service.h"
#include "app_scheduler.h"

#include "ble_dfu.h"
#include "gpio_driver.h"
#include "mpu6050.h"
#include "nrfx_timer.h"
#include "nrfx_twim.h"
#include "ppi_driver.h"
#include "pwm_driver.h"
#include "saadc_driver.h"
#include "twim_driver.h"

// Our Custom services objects
static ble_os_t ble_os1;
#if APP_ENABLE_SAADC || APP_ENABLE_TWIM
static ble_os_t ble_os2;
#endif
#if BLE_DFU_ENABLED
static ble_os_t ble_os_dfu;
#endif

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

#if APP_ENABLE_TWIM
bool scheduler_enabled = false;
#endif

// CustomService implementation

// <=================================== SERVICES ====================================
/**@brief Function to add characterstic to its service
 */

static void add_characteristic(ble_os_t *ble_os, ble_char_config_t *config) {
  ret_code_t err_code;
  ble_uuid_t char_uuid;
  ble_uuid128_t base_uuid = BLE_BASE_UUID;
  char_uuid.uuid          = config->uuid;
  err_code                = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);

  APP_ERROR_CHECK(err_code);

  ble_gatts_char_md_t char_md;
  memset(&char_md, 0, sizeof(char_md));
  char_md = config->char_md;

  // CCCD metadata (if notifications)
  ble_gatts_attr_md_t cccd_md;
  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  if (char_md.char_props.notify || char_md.char_props.indicate)
    char_md.p_cccd_md = &cccd_md;

  // Attribute metadata
  ble_gatts_attr_md_t attr_md = config->attr_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  // Attribute value structure
  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = config->init_len;
  attr_char_value.max_len   = config->max_len;
  attr_char_value.p_value   = config->init_value;

  // Add characteristic
  err_code = sd_ble_gatts_characteristic_add(ble_os->service_handle, &char_md, &attr_char_value, config->handle);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEBUG("Executing add_characteristic(), uuid:  0x%04X, type: 0x%02x, handle: 0x%02x",
                char_uuid.uuid,
                char_uuid.type,
                config->handle);
}

/**@brief Function to configure and add custom service to BLE profile table
 */

static void init_service(ble_os_t *ble_os, ble_os_config_t *os_config) {
  memset(ble_os, 0, sizeof(ble_os_t));

  ret_code_t err_code;
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = *(os_config->base_uuid);
  service_uuid.uuid       = os_config->service_uuid;
  ble_os->service_uuid    = os_config->service_uuid;

  // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
  err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
  APP_ERROR_CHECK(err_code);

  // Add our service

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &ble_os->service_handle);

  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEBUG("Service Init -> UUID: 0x%04X, Type: 0x%02X, Handle: 0x%02X",
                service_uuid.uuid,
                service_uuid.type,
                ble_os->service_handle);

  if (os_config->char1_config != NULL) {
    add_characteristic(ble_os, os_config->char1_config);
  }

  if (os_config->char2_config != NULL) {
    add_characteristic(ble_os, os_config->char2_config);
  }

  if (os_config->char3_config != NULL) {
    add_characteristic(ble_os, os_config->char3_config);
  }
}

#if BLE_DFU_ENABLED

// Dfu evt handler
static void ble_dfu_buttonless_evt_handler(ble_dfu_buttonless_evt_type_t event) {
  switch (event) {
  case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    NRF_LOG_INFO("Device is preparing to enter bootloader mode\r\n");
    break;
  case BLE_DFU_EVT_BOOTLOADER_ENTER:
    NRF_LOG_INFO("Device will enter bootloader mode\r\n");
    break;
  case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
    NRF_LOG_ERROR("Device will enter bootloader mode\r\n");
    break;
  default:
    NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
    break;
  }
}

// Initialization DFU services
static void init_dfu_service(void) {
  ble_dfu_buttonless_init_t dfus_init = {0};
  dfus_init.evt_handler               = ble_dfu_buttonless_evt_handler;

  uint32_t err_code = ble_dfu_buttonless_init(&dfus_init);
  APP_ERROR_CHECK(err_code);
}

#endif

// configure services and then adding
void init_services(void) {
  const ble_uuid128_t base_uuid = BLE_BASE_UUID;

  // OnBoards LED
  ble_char_config_t char1_config = {.uuid       = BLE_SERVICE1_CHAR1_UUID,
                                    .char_md    = {.char_props = {.write = 1, .read = 1}},
                                    .attr_md    = {.vloc = BLE_GATTS_VLOC_STACK},
                                    .init_value = (uint8_t[]){0x00},
                                    .init_len   = 2,
                                    .max_len    = 2,
                                    .handle     = &ble_os1.char1_handle};
  // OnBoard Buttons press notification
  ble_char_config_t char2_config = {.uuid       = BLE_SERVICE1_CHAR2_UUID,
                                    .char_md    = {.char_props = {.notify = 1, .read = 1}},
                                    .attr_md    = {.vloc = BLE_GATTS_VLOC_STACK},
                                    .init_value = (uint8_t[]){0x00},
                                    .init_len   = 1,
                                    .max_len    = 1,
                                    .handle     = &ble_os1.char2_handle};

  ble_char_config_t *char3_config = NULL;

#if APP_ENABLE_PWM
  // PWM
  ble_char_config_t char3_cfg = {.uuid       = BLE_SERVICE1_CHAR3_UUID,
                                 .char_md    = {.char_props = {.write = 1, .read = 1}},
                                 .attr_md    = {.vloc = BLE_GATTS_VLOC_STACK},
                                 .init_value = (uint8_t[]){0x00},
                                 .init_len   = 1,
                                 .max_len    = 1,
                                 .handle     = &ble_os1.char3_handle};
  char3_config                = &char3_cfg;
#endif

  // Custom service
  ble_os_config_t os_config = {.service_uuid = BLE_SERVICE1_UUID,
                               .base_uuid    = &base_uuid,
                               .char1_config = &char1_config,
                               .char2_config = &char2_config,
                               .char3_config = char3_config};
  // Init service
  init_service(&ble_os1, &os_config);

  memset(&os_config, 0, sizeof(os_config));

  ble_char_config_t *os2_char1_config = NULL;

#if APP_ENABLE_SAADC
  // ADC
  ble_char_config_t os2_char1_cfg = {.uuid       = BLE_SERVICE2_CHAR1_UUID,
                                     .char_md    = {.char_props = {.notify = 1, .read = 1}},
                                     .attr_md    = {.vloc = BLE_GATTS_VLOC_STACK},
                                     .init_value = NULL,
                                     .init_len   = sizeof(float),
                                     .max_len    = sizeof(float),
                                     .handle     = &ble_os2.char1_handle};
  os2_char1_config                = &os2_char1_cfg;
#endif

  ble_char_config_t *os2_char2_config = NULL;
#if APP_ENABLE_TWIM
  // TWI (I2c)

  ble_char_config_t os2_char2_cfg = {.uuid       = BLE_SERVICE2_CHAR2_UUID,
                                     .char_md    = {.char_props = {.notify = 1, .read = 1}},
                                     .attr_md    = {.vloc = BLE_GATTS_VLOC_STACK},
                                     .init_value = (uint8_t[]){0x00},
                                     .init_len   = sizeof(float),
                                     .max_len    = sizeof(float),
                                     .handle     = &ble_os2.char2_handle};

  os2_char2_config = &os2_char2_cfg;
#endif

#if APP_ENABLE_SAADC || APP_ENABLE_TWIM
  os_config = (ble_os_config_t){.service_uuid = BLE_SERVICE2_UUID,
                                .base_uuid    = &base_uuid,
                                .char1_config = os2_char1_config,
                                .char2_config = os2_char2_config,
                                .char3_config = NULL

  };
  // Init service
  init_service(&ble_os2, &os_config);
#endif

#if BLE_DFU_ENABLED

  // DFU service initialization
  init_dfu_service();
#endif
}

/**@brief Function to send data as notification to remote device
 */

static void notify_with_data(uint16_t char_handle, uint8_t *data, uint16_t len) {
  ble_gatts_hvx_params_t hvx_params;
  memset(&hvx_params, 0, sizeof(hvx_params));

  hvx_params.handle = char_handle;
  hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset = 0;
  hvx_params.p_len  = &len;
  hvx_params.p_data = data;

  ret_code_t err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_WARNING("sd_ble_gatts_hvx failed: 0x%08X (%d)", err_code, err_code);
  } else {
    NRF_LOG_DEBUG("Notification sent, handle=0x%04X, len=%u", char_handle, len);
  }
}

// <======================== TIMER1 ===========================>
//

#if APP_ENABLE_TIMER
// Timer1 Object
static const nrfx_timer_t my_timer1 = NRFX_TIMER_INSTANCE(1);

// Timer events handler
static void timer_evt_handler(nrf_timer_event_t event_type, void *p_context) {
  if (event_type == NRF_TIMER_EVENT_COMPARE0) {
#if APP_ENABLE_TWIM
    mpu6050_read_values();
#endif
  }
}

// Timer 1 initilization
static void timer_init(void) {
  ret_code_t err_code;

  nrfx_timer_config_t timer_config = {.frequency          = NRF_TIMER_FREQ_1MHz,
                                      .bit_width          = NRF_TIMER_BIT_WIDTH_32,
                                      .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
                                      .mode               = NRF_TIMER_MODE_TIMER,
                                      .p_context          = NULL};

  err_code = nrfx_timer_init(&my_timer1, &timer_config, timer_evt_handler);
  APP_ERROR_CHECK(err_code);

  // CHANNEL 0: Periodic using SHORT(Auto clear)
  uint32_t ticks_2000ms = nrfx_timer_ms_to_ticks(&my_timer1, 2000);
  nrfx_timer_extended_compare(&my_timer1,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks_2000ms,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,    // auto reset
                              true);

  // CHANNEL1 1: One shot without SHORT
  nrfx_timer_enable(&my_timer1);

  NRF_LOG_INFO("Mixed timer started:");
}

#endif

#if APP_ENABLE_TIMER
// Uninitlization timer on disconnect event
static void timer_uinit(void) {
  nrfx_timer_compare_int_disable(&my_timer1, NRF_TIMER_CC_CHANNEL0);
  nrfx_timer_disable(&my_timer1);
  nrfx_timer_clear(&my_timer1);
  nrfx_timer_uninit(&my_timer1);
}
#endif

//************************** Events Callbacks ********************************************

#if APP_ENABLE_TWIM
// MPU sesnor data event handler
static void mpu6050_data_handler(void *p_event_data, uint16_t event_size) {
  float *temperature = (float *)p_event_data;
  uint8_t encode_value[sizeof(float)];
  uint16_t len = sizeof(float);
  memcpy(encode_value, temperature, sizeof(float));
  notify_with_data(ble_os2.char2_handle.value_handle, encode_value, len);
}

// TWIM data event handler
static void i2c_event_handler(nrfx_twim_evt_t const *p_event) {
  NRF_LOG_ERROR("i2c_event_handler");
  if (p_event->type == NRFX_TWIM_EVT_DONE) {
    NRF_LOG_ERROR("i2c evt handler DONE");
    if (mpu6050_is_initialized() && ble_os2.char2_notification_enabled) {
      NRF_LOG_ERROR("i2c evt handler parsing");
      float temp = parse_mpu6050_data();
      NRF_LOG_DEBUG("Temp : " NRF_LOG_FLOAT_MARKER " °C", NRF_LOG_FLOAT(temp));
      // Schedule the event for processing in main context
      ret_code_t err_code;
      err_code = app_sched_event_put(&temp, sizeof(temp), mpu6050_data_handler);

      if (err_code != NRF_SUCCESS) {
        // Queue is full - handle error
        NRF_LOG_ERROR("Failed to schedule event: %d", err_code);
      }
    }
  }
}

#endif

#if APP_ENABLE_SAADC
// SAADC event handler
static void on_saadc_sample_evt(float value) {
  if (ble_os2.char1_notification_enabled) {
    uint8_t encode_value[sizeof(float)];
    uint16_t len = sizeof(float);
    memcpy(encode_value, &value, sizeof(float));
    notify_with_data(ble_os2.char1_handle.value_handle, encode_value, len);
  }
}

#endif

// Function to process writes events from BLE Stack
void on_write_evt(ble_evt_t const *ble_evt) {
  if (ble_evt == NULL) {
    NRF_LOG_DEBUG("ble_os or ble_evt is NULL");
    return;
  }

  const ble_gatts_evt_write_t *evt_write = &ble_evt->evt.gatts_evt.params.write;

  if (evt_write->handle == ble_os1.char1_handle.value_handle) {
    uint8_t num = evt_write->data[0];
    gpio_toggle(&num);
  } else if (evt_write->handle == ble_os1.char3_handle.value_handle) {
    uint8_t data = evt_write->data[0];
#if APP_ENABLE_PWM
    on_pwm_evt(data);
#endif

  }
#if APP_ENABLE_SAADC

  else if (evt_write->handle == ble_os2.char1_handle.cccd_handle) {
    if (evt_write->len == 2) {
      // Correct way to parse CCCD value (little-endian)
      uint16_t cccd_value = (evt_write->data[1] << 8) | evt_write->data[0];    // or *(uint16_t *)evt_write->data

      // Update notification status
      ble_os2.char1_notification_enabled = (cccd_value & BLE_GATT_HVX_NOTIFICATION);

      // Log the subscription change
      if (ble_os2.char1_notification_enabled) {
        NRF_LOG_INFO("Notifications SUBSCRIBED for handle: 0x%04X", ble_os2.char1_handle.cccd_handle);
      } else {
        NRF_LOG_INFO("Notifications UNSUBSCRIBED for handle: 0x%04X", ble_os2.char1_handle.cccd_handle);
      }

      NRF_LOG_DEBUG("CCCD value: 0x%04X", cccd_value);
    } else {
      NRF_LOG_WARNING("Invalid CCCD write length: %d (expected 2)", evt_write->len);
    }

  }
#endif

#if APP_ENABLE_TWIM

  else if (evt_write->handle == ble_os2.char2_handle.cccd_handle) {
    if (evt_write->len == 2) {
      // Correct way to parse CCCD value (little-endian)
      uint16_t cccd_value = (evt_write->data[1] << 8) | evt_write->data[0];    // or *(uint16_t *)evt_write->data

      // Update notification status
      ble_os2.char2_notification_enabled = (cccd_value & BLE_GATT_HVX_NOTIFICATION);

      // Log the subscription change
      if (ble_os2.char2_notification_enabled) {
        NRF_LOG_INFO("Notifications SUBSCRIBED for handle: 0x%04X", ble_os2.char2_handle.cccd_handle);
      } else {
        NRF_LOG_INFO("Notifications UNSUBSCRIBED for handle: 0x%04X", ble_os2.char2_handle.cccd_handle);
      }

      NRF_LOG_DEBUG("CCCD value: 0x%04X", cccd_value);
    } else {
      NRF_LOG_WARNING("Invalid CCCD write length: %d (expected 2)", evt_write->len);
    }
  }
#endif

  NRF_LOG_DEBUG("on_write_evt called, handle : 0x%04X", evt_write->handle);
}

// @brief Onboard Button press event handler
void on_button_evt(bsp_event_t evt, uint16_t *conn_handle) {
  uint16_t len = 1;
  ble_gatts_hvx_params_t hvx_params;
  memset(&hvx_params, 0, sizeof(hvx_params));
  uint8_t value = evt == BSP_EVENT_KEY_1 ? 0x01 : evt == BSP_EVENT_KEY_2 ? 0x02 : 0x03;

  hvx_params.handle   = ble_os1.char2_handle.value_handle,    //*char_value_handle;
      hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset   = 0;
  hvx_params.p_len    = &len;
  hvx_params.p_data   = &value;

  sd_ble_gatts_hvx(*conn_handle, &hvx_params);
}

// Order is importent
void on_connected(uint16_t conn_handle) {
  m_conn_handle = conn_handle;

#if APP_ENABLE_PWM
  pwm_init();
#endif

#if APP_ENABLE_SAADC && APP_ENABLE_TIMER
  // Initialize SAADC first (endpoint)
  saadc_init(on_saadc_sample_evt);
  // Now create PPI link
  ppi_saadc_init(&my_timer1);
#endif

#if APP_ENABLE_TIMER
  // Initialize timer next
  timer_init();
#endif

#if APP_ENABLE_TWIM && APP_ENABLE_TIMER
  i2c_init(I2C_SCL_PIN, I2C_SDA_PIN, i2c_event_handler);
  // Now create PPI link
  ppi_twim_init(&my_timer1, &i2c);
  // MPU6050 initlize
  mpu6050_init();
  mpu6050_read_values();
  scheduler_enabled = true;
#endif

  NRF_LOG_INFO("cus_init completed");
}

void on_disconnected(void) {
  m_conn_handle = BLE_CONN_HANDLE_INVALID;
  // 1) Disable PPI first
  ppi_uninit();

#if APP_ENABLE_TIMER
  // 2) Stop timer
  timer_uinit();
#endif

#if APP_ENABLE_SAADC
  // 3) Stop SAADC safely
  saadc_uinit();
#endif

  // 4) PWM
#if APP_ENABLE_PWM
  pwm_uinit();
#endif

#if APP_ENABLE_TWIM
  // TWIM(I2C)
  i2c_uninit();
  scheduler_enabled = false;
#endif

  NRF_LOG_INFO("cus_uinit completed");
}