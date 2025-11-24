/**
 * Author : Wisal Muhammad
 * Date : 19/11/2025
 * @file mpu6050.c
 * @brief MPU6050 IMU (Accelerometer + Gyroscope + Temp Sensor) driver implementation.
 *
 * Provides initialization, data reading, and raw data parsing functions for the
 * MPU6050 sensor using the I2C (TWIM) driver.
 *
 * The driver uses nrfx TWIM and requires `APP_ENABLE_TWIM` to be enabled.
 */

#include "mpu6050.h"
#include "nrf_delay.h"
#include "twim_driver.h"

#if APP_ENABLE_TWIM

// Tracks whether the MPU6050 has been successfully initialized.
static bool mpu6050_initialized = false;

/// Raw sensor data buffer (Accel + Temp + Gyro = 14 bytes)
static uint8_t sensor_data[14] = {0x00, 0x01};

/**
 * @brief Check whether the MPU6050 sensor is initialized.
 *
 * @return true  If initialization was successful
 * @return false If sensor not initialized yet
 */

bool mpu6050_is_initialized(void) {
  return mpu6050_initialized;
}

/**
 * @brief Initialize the MPU6050 sensor using I2C (TWIM).
 *
 * Sequence:
 *  - Wake up the sensor (exit sleep mode)
 *  - Read WHO_AM_I register to confirm sensor identity
 *  - Set internal state to initialized if successful
 *
 * This function must be called before reading sensor values.
 */
void mpu6050_init(void) {
  mpu6050_initialized = false;
  ret_code_t err_code;
  uint8_t wake_data[2] = {PWR_MGMT_REG, 0x00};

  err_code = i2c_tx(MPU6050_ADDR, wake_data, sizeof(wake_data));

  if (err_code != NRF_SUCCESS) {
    NRF_LOG_DEBUG("Sensor wake up failed");
    return;
  }

  while (!i2c_transfer_done())
    ;

  uint8_t reg = WHO_AM_I_REG;
  uint8_t who_am_i;
  err_code = i2c_tx_rx(MPU6050_ADDR, &reg, sizeof(reg), &who_am_i, sizeof(who_am_i), 0);    // 0 for default settings
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_DEBUG("Error identify sesnor");
    return;
  }

  while (!i2c_transfer_done())
    ;
  // m_xfer_done = false;

  if (who_am_i != 0x68) {
    NRF_LOG_WARNING("MPU6050 WHO_AM_I mismatch: 0x%02X (expected 0x68)", who_am_i);
    return;
  }

  mpu6050_initialized = true;

  // prepare_xfer_desc();
  NRF_LOG_INFO("Sensor identified: 0x%02X", who_am_i);
}

/**
 * @brief Parse the raw sensor buffer into meaningful physical values.
 *
 * Converts:
 *  - Accelerometer values → g units
 *  - Gyroscope values → °/s
 *  - Temperature → °C
 *
 * @return float The temperature value in °C
 */
float parse_mpu6050_data(void) {
  int16_t accel_x  = (sensor_data[0] << 8) | sensor_data[1];
  int16_t accel_y  = (sensor_data[2] << 8) | sensor_data[3];
  int16_t accel_z  = (sensor_data[4] << 8) | sensor_data[5];
  int16_t temp_raw = (sensor_data[6] << 8) | sensor_data[7];
  int16_t gyro_x   = (sensor_data[8] << 8) | sensor_data[9];
  int16_t gyro_y   = (sensor_data[10] << 8) | sensor_data[11];
  int16_t gyro_z   = (sensor_data[12] << 8) | sensor_data[13];

  float temperature = (temp_raw / 340.0f) + 36.53f;
  float ax          = accel_x / 16384.0f;
  float ay          = accel_y / 16384.0f;
  float az          = accel_z / 16384.0f;
  float gx          = gyro_x / 131.0f;
  float gy          = gyro_y / 131.0f;
  float gz          = gyro_z / 131.0f;

  NRF_LOG_INFO("Accel X=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(ax));
  NRF_LOG_INFO("Accel Y=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(ay));
  NRF_LOG_INFO("Accel Z=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(az));

  NRF_LOG_INFO("Gyro X=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gx));
  NRF_LOG_INFO("Gyro Y=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gy));
  NRF_LOG_INFO("Gyro Z=" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(gz));

  NRF_LOG_INFO("Temperature = " NRF_LOG_FLOAT_MARKER " °C", NRF_LOG_FLOAT(temperature));

  return temperature;
}

/**
 * @brief Trigger a read of all MPU6050 sensor registers (Accelerometer, Temp, Gyro).
 *
 * Reads 14 consecutive bytes starting at ACCEL_XOUT_H.
 * The parsed values can be accessed via `parse_mpu6050_data()`.
 */

void mpu6050_read_values(void) {
  // Re-arm the xfer_desc for the next trigger
  ret_code_t err_code;
  if (mpu6050_initialized) {
    uint8_t start_reg = ACCEL_XOUT_H;
    err_code = i2c_tx_rx(MPU6050_ADDR, &start_reg, sizeof(start_reg), sensor_data, sizeof(sensor_data), 0);

    if (err_code != NRF_SUCCESS) {
      NRF_LOG_DEBUG("Error reading sesnor data");
      return;
    }

  } else {
    NRF_LOG_ERROR("MPU6050 not initilized")
  }

}
#endif
