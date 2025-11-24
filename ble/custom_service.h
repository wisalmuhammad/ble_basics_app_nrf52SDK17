/**
 * @file custom_service.h
 * @brief Custom BLE service definitions and configuration structures.
 *
 * This file defines the data structures and function prototypes required
 * to initialize and operate a custom BLE service containing up to three
 * characteristics. It includes configuration structures for flexible
 * characteristic creation, service runtime state, and event handler APIs
 * for BLE connection, disconnection, write events, and application-level
 * callbacks.
 *
 * Features:
 * - Creates a custom BLE service with up to 3 configurable characteristics.
 * - Supports notifications per characteristic.
 * - Integrates optional TWIM-based scheduling synchronization.
 * - Provides event handlers for BLE writes, button input, and PWM updates.
 *
 * @date 19/11/2025
 * @author Wisal Muhammad
 */

#ifndef CUSTOM_SERVICE_H
#define CUSTOM_SERVICE_H

#include "stdint.h"
#include "ble.h"
#include "ble_uuid.h"
#include "bsp.h"

#if  APP_ENABLE_TWIM
/** @brief Indicates whether the scheduler is enabled (extern if using TWIM). */
extern bool scheduler_enabled;
#endif

/**
 * @brief Runtime state for the custom BLE service.
 *
 * This structure holds handles assigned by the SoftDevice as well as
 * runtime flags indicating whether notifications are enabled for each
 * characteristic.
 */
typedef struct{

  uint16_t                   service_handle;
  uint16_t                   service_uuid;   //TODO to Remove
  ble_gatts_char_handles_t   char1_handle;
  ble_gatts_char_handles_t   char2_handle;
  ble_gatts_char_handles_t   char3_handle;
  
  bool                       char1_notification_enabled;
  bool                       char2_notification_enabled;
  bool                       char3_notification_enabled;

} ble_os_t ;

/**
 * @brief Configuration structure for a single BLE characteristic.
 *
 * Use this structure when setting up a characteristic during service initialization.
 */
typedef struct {
  uint16_t uuid;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t attr_md;
  uint8_t *init_value;
  uint8_t init_len;
  uint16_t max_len;
  ble_gatts_char_handles_t *handle;
} ble_char_config_t;

/**
 * @brief Configuration object for setting up the custom BLE service.
 *
 * Contains service UUID, optional 128-bit base UUID, and up to three
 * characteristic configuration blocks.
 */
typedef struct {
  
  uint16_t              service_uuid;
  ble_uuid128_t const*  base_uuid;
  ble_char_config_t    *char1_config;
  ble_char_config_t    *char2_config;
  ble_char_config_t    *char3_config;    

} ble_os_config_t;


/**
 * @brief Called on BLE connection event.
 *
 * @param conn_handle  Handle of the active connection.
 */
void on_connected(uint16_t conn_handle);

/**
 * @brief Called on BLE disconnection event.
 */
void on_disconnected(void);

/**
 * @brief Initializes the custom BLE service and its characteristics.
 */
void init_services(void);

/**
 * @brief Handles button events and may send notifications depending on active connection.
 *
 * @param evt          Button event (BSP event).
 * @param conn_handle  Pointer to the active BLE connection handle.
 */
void on_button_evt(bsp_event_t evt, uint16_t *conn_handle);

/**
 * @brief BLE write event handler for characteristics in this service.
 *
 * @param ble_evt  Pointer to BLE event containing write data.
 */
void on_write_evt(ble_evt_t const *ble_evt);

/**
 * @brief Application callback for PWM updates (example use-case).
 *
 * @param duty_cycle new PWM duty cycle value.
 */
void on_pwm_evt(uint8_t duty_cycle);

#endif // CUSTOM_SERVICE_H