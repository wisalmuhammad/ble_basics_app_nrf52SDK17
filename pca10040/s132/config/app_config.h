/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// <h> nRF_Log

//==========================================================
// <e> NRF_LOG_BACKEND_RTT_ENABLED - nrf_log_backend_rtt - Log RTT backend
//==========================================================
#ifndef NRF_LOG_BACKEND_RTT_ENABLED
#define NRF_LOG_BACKEND_RTT_ENABLED 1
#endif

// <e> NRF_LOG_ENABLED - nrf_log - Logger
//==========================================================
#ifndef NRF_LOG_ENABLED
#define NRF_LOG_ENABLED 1
#endif

// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug

#ifndef NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL 4
#endif

// <o> NRF_SDH_BLE_VS_UUID_COUNT - The number of vendor-specific UUIDs.
#ifndef NRF_SDH_BLE_VS_UUID_COUNT
#define NRF_SDH_BLE_VS_UUID_COUNT 2
#endif

// Enabling PWM working in the app
#ifndef APP_ENABLE_PWM
#define APP_ENABLE_PWM 1
#endif

// <e> NRFX_PWM_ENABLED - nrfx_pwm - PWM peripheral driver
//==========================================================

#if APP_ENABLE_PWM
#ifndef NRFX_PWM_ENABLED
#define NRFX_PWM_ENABLED 1
#endif
// <q> NRFX_PWM0_ENABLED  - Enable PWM0 instance

#ifndef NRFX_PWM0_ENABLED
#define NRFX_PWM0_ENABLED 1
#endif

// <e> PWM_ENABLED - nrf_drv_pwm - PWM peripheral driver - legacy layer
//==========================================================
#ifndef PWM_ENABLED
#define PWM_ENABLED 1
#endif

// <q> PWM0_ENABLED  - Enable PWM0 instance

#ifndef PWM0_ENABLED
#define PWM0_ENABLED 1
#endif

#endif


#ifndef APP_ENABLE_SAADC
#define APP_ENABLE_SAADC 1
#endif

// <e> SAADC_ENABLED - nrf_drv_saadc - SAADC peripheral driver - legacy layer
//==========================================================

#if APP_ENABLE_SAADC


#ifndef SAADC_ENABLED
#define SAADC_ENABLED 1
#endif

// <o> SAADC_CONFIG_RESOLUTION  - Resolution

// <0=> 8 bit
// <1=> 10 bit
// <2=> 12 bit
// <3=> 14 bit

#ifndef SAADC_CONFIG_RESOLUTION
#define SAADC_CONFIG_RESOLUTION 2
#endif

// <e> NRFX_SAADC_ENABLED - nrfx_saadc - SAADC peripheral driver
//==========================================================
#ifndef NRFX_SAADC_ENABLED
#define NRFX_SAADC_ENABLED 1
#endif
// <o> NRFX_SAADC_CONFIG_RESOLUTION  - Resolution

// <0=> 8 bit
// <1=> 10 bit
// <2=> 12 bit
// <3=> 14 bit

#ifndef NRFX_SAADC_CONFIG_RESOLUTION
#define NRFX_SAADC_CONFIG_RESOLUTION 2
#endif


#endif


#ifndef  APP_ENABLE_TIMER
#define  APP_ENABLE_TIMER 1
#endif


// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver - legacy layer
//==========================================================

#if APP_ENABLE_TIMER

#ifndef TIMER_ENABLED
#define TIMER_ENABLED 1
#endif

// <q> TIMER0_ENABLED  - Enable TIMER0 instance

#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 1
#endif


// <e> NRFX_TIMER_ENABLED - nrfx_timer - TIMER periperal driver
//==========================================================
#ifndef NRFX_TIMER_ENABLED
#define NRFX_TIMER_ENABLED 1
#endif
// <q> NRFX_TIMER0_ENABLED  - Enable TIMER0 instance

#ifndef NRFX_TIMER1_ENABLED
#define NRFX_TIMER1_ENABLED 1
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority

// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7

#ifndef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 7
#endif

// <q> PPI_ENABLED  - nrf_drv_ppi - PPI peripheral driver - legacy layer

#endif

#ifndef PPI_ENABLED
#define PPI_ENABLED 1
#endif

// <e> NRFX_PPI_ENABLED - nrfx_ppi - PPI peripheral allocator
//==========================================================
#ifndef NRFX_PPI_ENABLED
#define NRFX_PPI_ENABLED 1
#endif

// <e> NRFX_PPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PPI_CONFIG_LOG_ENABLED
#define NRFX_PPI_CONFIG_LOG_ENABLED 0
#endif

// Note that the following should be commeted out before uisng NRFX_x_x version

/*
//// <e> TWI_ENABLED - nrf_drv_twi - TWI/TWIM peripheral driver - legacy layer
////==========================================================
//#ifndef TWI_ENABLED
//#define TWI_ENABLED 0
//#endif
//// <o> TWI_DEFAULT_CONFIG_FREQUENCY  - Frequency

//// <26738688=> 100k
//// <67108864=> 250k
//// <104857600=> 400k

//#ifndef TWI_DEFAULT_CONFIG_FREQUENCY
//#define TWI_DEFAULT_CONFIG_FREQUENCY 26738688
//#endif

//// <q> TWI_DEFAULT_CONFIG_CLR_BUS_INIT  - Enables bus clearing procedure during init


//#ifndef TWI_DEFAULT_CONFIG_CLR_BUS_INIT
//#define TWI_DEFAULT_CONFIG_CLR_BUS_INIT 0
//#endif

//// <q> TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT  - Enables bus holding after uninit


//#ifndef TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
//#define TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
//#endif

//// <o> TWI_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority


//// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//// <0=> 0 (highest)
//// <1=> 1
//// <2=> 2
//// <3=> 3
//// <4=> 4
//// <5=> 5
//// <6=> 6
//// <7=> 7

//#ifndef TWI_DEFAULT_CONFIG_IRQ_PRIORITY
//#define TWI_DEFAULT_CONFIG_IRQ_PRIORITY 6
//#endif

//// <e> TWI0_ENABLED - Enable TWI0 instance
////==========================================================
//#ifndef TWI0_ENABLED
//#define TWI0_ENABLED 0
//#endif
//// <q> TWI0_USE_EASY_DMA  - Use EasyDMA (if present)


//#ifndef TWI0_USE_EASY_DMA
//#define TWI0_USE_EASY_DMA 0
//#endif

//// </e>

//// <e> TWI1_ENABLED - Enable TWI1 instance
////==========================================================
//#ifndef TWI1_ENABLED
//#define TWI1_ENABLED 0
//#endif
//// <q> TWI1_USE_EASY_DMA  - Use EasyDMA (if present)


//#ifndef TWI1_USE_EASY_DMA
//#define TWI1_USE_EASY_DMA 0
//#endif

//// </e>

//// <q> TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED  - Enables nRF52 anomaly 109 workaround for TWIM.


//// <i> The workaround uses interrupts to wake up the CPU by catching
//// <i> the start event of zero-frequency transmission, clear the
//// <i> peripheral, set desired frequency, start the peripheral, and
//// <i> the proper transmission. See more in the Errata document or
//// <i> Anomaly 109 Addendum located at https://infocenter.nordicsemi.com/

//#ifndef TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED
//#define TWIM_NRF52_ANOMALY_109_WORKAROUND_ENABLED 0
//#endif


*/


#ifndef APP_ENABLE_TWIM
#define APP_ENABLE_TWIM  1
#endif


// <e> NRFX_TWIM_ENABLED - nrfx_twim - TWIM peripheral driver
//==========================================================

#if APP_ENABLE_TWIM

#ifndef NRFX_TWIM_ENABLED
#define NRFX_TWIM_ENABLED 1
#endif
// <q> NRFX_TWIM0_ENABLED  - Enable TWIM0 instance

#ifndef NRFX_TWIM0_ENABLED
#define NRFX_TWIM0_ENABLED 1
#endif

#endif

//==========================================================
// <e> APP_SCHEDULER_ENABLED - app_scheduler - Events scheduler
//==========================================================
#ifndef APP_SCHEDULER_ENABLED
#define APP_SCHEDULER_ENABLED 1
#endif

//==========================================================


// <h> nRF_DFU

//==========================================================
// <h> ble_dfu - Device Firmware Update

//==========================================================
// <q> BLE_DFU_ENABLED  - Enable DFU Service.


#define NRF_DFU_TRANSPORT_BLE 1

#ifndef BLE_DFU_ENABLED
#define BLE_DFU_ENABLED 1
#endif

// <q> NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS  - Buttonless DFU supports bonds.

#ifndef NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS
#define NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS 0
#endif

// <q> NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY  - Blocked shutdown procedure will be retried every second.

#ifndef NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY
#define NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY 1
#endif

// <q> NRF_SDH_BLE_SERVICE_CHANGED  - Include the Service Changed characteristic in the Attribute Table.

#ifndef NRF_SDH_BLE_SERVICE_CHANGED
#define NRF_SDH_BLE_SERVICE_CHANGED 1
#endif


// <<< end of configuration section >>>
#endif    // APP_CONFIG_H
