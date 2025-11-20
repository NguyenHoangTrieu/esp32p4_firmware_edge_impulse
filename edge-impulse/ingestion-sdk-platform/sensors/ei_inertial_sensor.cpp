/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "model-parameters/model_metadata.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/porting/ei_logging.h"
#include "ei_inertial_sensor.h"
#include <stdint.h>
#include <string.h>

// ESP-IDF I2C driver
#include "driver/i2c_master.h"
#include "esp_err.h"

/* MPU6050 Registers ------------------------------------------------------- */
#define MPU6050_ADDR 0x68 // Default I2C address
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

/* Configuration ----------------------------------------------------------- */
#define I2C_MASTER_SCL_IO 8      // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 7      // GPIO number for I2C SDA
#define I2C_MASTER_FREQ_HZ 400000 // I2C master clock frequency (400kHz)
#define I2C_MASTER_TIMEOUT_MS 1000

#define CONVERT_G_TO_MS2 9.80665f
#define MPU6050_ACCEL_SCALE 16384.0f // For ±2g range

/* Static variables -------------------------------------------------------- */
static float imu_data[INERTIAL_AXIS_SAMPLED];
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_handle = NULL;

/* I2C Helper Functions ---------------------------------------------------- */

/**
 * @brief Write a byte to MPU6050 register
 */
static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_transmit(mpu6050_handle, write_buf, sizeof(write_buf),
                             I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Read multiple bytes from MPU6050 register
 */
static esp_err_t mpu6050_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
  return i2c_master_transmit_receive(mpu6050_handle, &reg_addr, 1, data, len,
                                     I2C_MASTER_TIMEOUT_MS);
}

/* Public Functions -------------------------------------------------------- */

/**
 * @brief Initialize MPU6050 sensor
 */
bool ei_inertial_init(void) {
  esp_err_t ret;
  uint8_t who_am_i = 0;

  // Configure I2C master bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
      .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags =
          {
              .enable_internal_pullup = true,
          },
  };

  ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
  if (ret != ESP_OK) {
    EI_LOGE("Failed to initialize I2C master bus: %s\n", esp_err_to_name(ret));
    return false;
  }

  // Configure MPU6050 device
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };

  ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &mpu6050_handle);
  if (ret != ESP_OK) {
    EI_LOGE("Failed to add MPU6050 device: %s\n", esp_err_to_name(ret));
    i2c_del_master_bus(i2c_bus_handle);
    return false;
  }

  // Wait for sensor to stabilize
  ei_sleep(100);

  // Check WHO_AM_I register
  ret = mpu6050_read_reg(MPU6050_WHO_AM_I, &who_am_i, 1);
  if (ret != ESP_OK || who_am_i != 0x68) {
    EI_LOGW("Failed to connect to MPU6050 (WHO_AM_I=0x%02X)!\n", who_am_i);
    i2c_master_bus_rm_device(mpu6050_handle);
    i2c_del_master_bus(i2c_bus_handle);
    return false;
  }

  EI_LOGI("MPU6050 detected (WHO_AM_I=0x%02X)\n", who_am_i);

  // Wake up MPU6050 (write 0 to PWR_MGMT_1)
  ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
  if (ret != ESP_OK) {
    EI_LOGE("Failed to wake up MPU6050\n");
    return false;
  }

  ei_sleep(10);

  // Configure accelerometer: ±2g full scale range
  ret = mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
  if (ret != ESP_OK) {
    EI_LOGE("Failed to configure MPU6050 accelerometer\n");
    return false;
  }

  // Register sensor with Edge Impulse
  if (ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
    EI_LOGE("Failed to register Inertial sensor!\n");
    return false;
  }

  EI_LOGI("MPU6050 initialized successfully\n");
  return true;
}

/**
 * @brief Read accelerometer data from MPU6050
 */
float *ei_fusion_inertial_read_data(int n_samples) {
  uint8_t raw_data[6];
  int16_t accel_x, accel_y, accel_z;
  esp_err_t ret;

  // Read 6 bytes starting from ACCEL_XOUT_H
  ret = mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, raw_data, 6);
  if (ret != ESP_OK) {
    EI_LOGE("Failed to read accelerometer data\n");
    // Return previous data on error
    return imu_data;
  }

  // Combine high and low bytes (big-endian)
  accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

  // Convert to m/s² (from raw value to g, then to m/s²)
  imu_data[0] = ((float)accel_x / MPU6050_ACCEL_SCALE) * CONVERT_G_TO_MS2;
  imu_data[1] = ((float)accel_y / MPU6050_ACCEL_SCALE) * CONVERT_G_TO_MS2;
  imu_data[2] = ((float)accel_z / MPU6050_ACCEL_SCALE) * CONVERT_G_TO_MS2;

  return imu_data;
}

/**
 * @brief Sample callback for standalone accelerometer sampling
 */
bool ei_inertial_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    EI_LOGI("Starting accelerometer sampling...\n");
    
    // Sample loop
    while (1) {
        float *buffer = ei_fusion_inertial_read_data(INERTIAL_AXIS_SAMPLED);
        
        // Send data to Edge Impulse via callback
        callsampler((const void *)buffer, INERTIAL_AXIS_SAMPLED * sizeof(float));
        
        // Wait for next sample
        ei_sleep((int)sample_interval_ms);
        
        // Check for stop signal
        if (ei_user_invoke_stop()) {
            EI_LOGI("Sampling stopped by user\n");
            break;
        }
    }
    
    return true;
}
