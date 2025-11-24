/**
 * @file mpu6050.h
 * @brief MPU6050 accelerometer/gyroscope driver interface.
 *
 * This module provides function declarations and register definitions for
 * interfacing with the InvenSense MPU6050 sensor using I2C (TWIM).
 * It handles initialization, sensor identity verification, raw data acquisition,
 * and parsing of accelerometer, gyroscope, and temperature readings.
 *
 * The driver depends on a functional TWIM (I2C) implementation and should be
 * initialized before reading sensor values.
 *
 * Supported features:
 * - Wakeup and configuration of the MPU6050
 * - WHO_AM_I identity verification
 * - Reading raw 14-byte sensor output frame
 * - Parsing sensor data into human-readable floating-point values
 *
 * I2C pin configuration:
 *  - SCL: @ref I2C_SCL_PIN
 *  - SDA: @ref I2C_SDA_PIN
 *
 * @author
 *   Wisal Muhammad
 *
 * @date
 *   19/11/2025
 */
#ifndef MPU_6050_H_
#define MPU_6050_H_

#include "stdint.h"
#include "stdbool.h"
#include "nrf_log.h"
#include "sdk_errors.h"
#include "ble.h"


/** @brief I2C device address of MPU6050. */
#define MPU6050_ADDR 0x68

/** @brief Register address for WHO_AM_I identity check. */
#define WHO_AM_I_REG 0x75

/** @brief Power management register (used to wake sensor). */
#define PWR_MGMT_REG 0x6B

/** @brief Starting register address for accelerometer readings. */
#define ACCEL_XOUT_H 0x3B

/** @brief Starting register address for gyroscope readings. */
#define GYRO_XOUT_H  0x43

/** @brief I2C SCL pin used for MPU6050 communication. */
#define I2C_SCL_PIN 27

/** @brief I2C SDA pin used for MPU6050 communication. */
#define I2C_SDA_PIN 26

/**
 * @brief Callback type for MPU6050 initialization completion.
 */
typedef void (*mpu6050_init_cb_t)(void);

/**
 * @brief Initializes the MPU6050 sensor.
 *
 * - Wakes the sensor from sleep mode.
 * - Verifies WHO_AM_I register.
 * - Sets ready flag upon successful initialization.
 */
void mpu6050_init(void);

/**
 * @brief Checks if MPU6050 was successfully initialized.
 *
 * @return true     Sensor is initialized.
 * @return false    Sensor is not initialized.
 */
bool mpu6050_is_initialized(void);

/**
 * @brief Triggers a read of the 14-byte sensor output frame.
 *
 * The data includes:
 * - Accel X, Y, Z
 * - Temperature
 * - Gyro X, Y, Z
 */
void mpu6050_read_values(void);

/**
 * @brief Parses previously read MPU6050 raw data.
 *
 * Converts accelerometer, gyroscope, and temperature raw values into
 * floating-point readings following the datasheet scale factors.
 *
 * @return Temperature in °C.
 */
float parse_mpu6050_data(void);

#endif /* MPU_6050_H_ */