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

#include "ei_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr_types.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OV5647";

// OV5647 I2C definitions
#define OV5647_I2C_ADDR 0x36
#define OV5647_CHIP_ID_H 0x300A
#define OV5647_CHIP_ID_L 0x300B
#define OV5647_CHIP_ID_VALUE 0x5647
#define OV5647_SW_RESET 0x0103
#define OV5647_SW_STANDBY 0x0100

// I2C configuration
#define I2C_MASTER_FREQ_HZ 100000 // 100kHz for OV5647
#define I2C_MASTER_TIMEOUT_MS 1000

// Static handles for I2C
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t ov5647_dev_handle = NULL;

// CSI transaction structure
static esp_cam_ctlr_trans_t s_trans = {};

/* Resolution configurations */
ei_device_snapshot_resolutions_t EiCameraESP32P4::resolutions[] = {
    {.width = 160, .height = 120},
    {.width = 320, .height = 240}, // QVGA
    {.width = 640, .height = 480}, // VGA - Recommended
    {.width = 800, .height = 600}  // 720p
};

EiCameraESP32P4::EiCameraESP32P4()
    : width(640), height(480), output_width(640), output_height(480),
      camera_present(false), cam_handle(NULL), frame_buffer(NULL),
      frame_buffer_size(0) {}

static bool on_trans_finished_cb(esp_cam_ctlr_handle_t handle,
                                 esp_cam_ctlr_trans_t *trans, void *user_data) {
  ESP_LOGD("CSI", "Frame received: %d bytes", trans->received_size);
  return false;
}

bool EiCameraESP32P4::is_camera_present(void) { return camera_present; }

ei_device_snapshot_resolutions_t EiCameraESP32P4::get_min_resolution(void) {
  return resolutions[0];
}

void EiCameraESP32P4::get_resolutions(ei_device_snapshot_resolutions_t **res,
                                      uint8_t *res_num) {
  *res = &EiCameraESP32P4::resolutions[0];
  *res_num = sizeof(EiCameraESP32P4::resolutions) /
             sizeof(ei_device_snapshot_resolutions_t);
}

bool EiCameraESP32P4::set_resolution(
    const ei_device_snapshot_resolutions_t res) {
  width = res.width;
  height = res.height;
  ESP_LOGI(TAG, "Resolution set to %dx%d", width, height);
  return true;
}

/* I2C Helper Functions for OV5647 - NEW API */

static bool ov5647_read_reg(uint16_t reg_addr, uint8_t *data) {
  if (!ov5647_dev_handle) {
    ESP_LOGE(TAG, "OV5647 device not initialized");
    return false;
  }

  uint8_t reg_buf[2] = {(uint8_t)((reg_addr >> 8) & 0xFF),
                        (uint8_t)(reg_addr & 0xFF)};

  // Write register address then read data
  esp_err_t ret =
      i2c_master_transmit_receive(ov5647_dev_handle, reg_buf, sizeof(reg_buf),
                                  data, 1, I2C_MASTER_TIMEOUT_MS);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C read reg 0x%04X failed: %s", reg_addr,
             esp_err_to_name(ret));
    return false;
  }

  return true;
}

static bool ov5647_write_reg(uint16_t reg_addr, uint8_t data) {
  if (!ov5647_dev_handle) {
    ESP_LOGE(TAG, "OV5647 device not initialized");
    return false;
  }

  uint8_t buf[3] = {(uint8_t)((reg_addr >> 8) & 0xFF),
                    (uint8_t)(reg_addr & 0xFF), data};

  esp_err_t ret = i2c_master_transmit(ov5647_dev_handle, buf, sizeof(buf),
                                      I2C_MASTER_TIMEOUT_MS);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C write reg 0x%04X failed: %s", reg_addr,
             esp_err_to_name(ret));
    return false;
  }

  return true;
}

static bool ov5647_init_sensor(uint16_t width, uint16_t height) {
  ESP_LOGI(TAG, "Initializing OV5647 sensor for %dx%d", width, height);

  // Software reset
  if (!ov5647_write_reg(0x0103, 0x01)) {
    ESP_LOGE(TAG, "Failed to reset OV5647");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // Software standby ON (stop streaming)
  if (!ov5647_write_reg(0x0100, 0x00)) {
    ESP_LOGE(TAG, "Failed to enter standby");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));

  // ===== PLL and Clock Settings =====
  ov5647_write_reg(0x0103, 0x01); // Software reset
  vTaskDelay(pdMS_TO_TICKS(10));

  ov5647_write_reg(0x3034, 0x1a); // MIPI 10-bit mode
  ov5647_write_reg(0x3035, 0x21); // System clock divider
  ov5647_write_reg(0x3036, 0x69); // PLL multiplier (105)
  ov5647_write_reg(0x303c, 0x11); // PLLS control
  ov5647_write_reg(0x3106, 0xf5); // System root divider

  // ===== CRITICAL: MIPI Control =====
  ov5647_write_reg(0x4800, 0x24); // MIPI control: 2-lane, gate clock
  ov5647_write_reg(0x4837, 0x16); // MIPI pclk period
  ov5647_write_reg(0x300e, 0x45); // MIPI virtual channel 0
  ov5647_write_reg(0x4814, 0x2b); // MIPI virtual channel, lane config

  // CRITICAL: Enable MIPI lanes
  ov5647_write_reg(0x300a, 0xff); // Lane enable
  ov5647_write_reg(0x300b, 0xff); // Lane enable

  // ===== System Control =====
  ov5647_write_reg(0x3000, 0x00);
  ov5647_write_reg(0x3001, 0x00);
  ov5647_write_reg(0x3002, 0x00);
  ov5647_write_reg(0x3016, 0x08);
  ov5647_write_reg(0x3017, 0xe0);
  ov5647_write_reg(0x3018, 0x44);
  ov5647_write_reg(0x301c, 0xf8);
  ov5647_write_reg(0x301d, 0xf0);

  // ===== Format Control =====
  ov5647_write_reg(0x4300, 0x60); // Format: RAW8 Bayer

  // ===== ISP Control =====
  ov5647_write_reg(0x5000, 0x06); // ISP: lenc on
  ov5647_write_reg(0x5001, 0x01); // Manual AWB
  ov5647_write_reg(0x5002, 0x00); // Win control off

  // ===== BLC =====
  ov5647_write_reg(0x4000, 0x09);
  ov5647_write_reg(0x4001, 0x02);
  ov5647_write_reg(0x4002, 0xc5);

  // ===== Analog Control =====
  ov5647_write_reg(0x3620, 0x52);
  ov5647_write_reg(0x3621, 0xe0);
  ov5647_write_reg(0x3622, 0x01);
  ov5647_write_reg(0x3630, 0x2e);
  ov5647_write_reg(0x3631, 0x00);
  ov5647_write_reg(0x3632, 0xe2);
  ov5647_write_reg(0x3633, 0x23);
  ov5647_write_reg(0x3634, 0x44);
  ov5647_write_reg(0x3636, 0x06);
  ov5647_write_reg(0x3704, 0xa0);
  ov5647_write_reg(0x3705, 0x1a);
  ov5647_write_reg(0x3708, 0x64);
  ov5647_write_reg(0x3709, 0x52);

  // ===== Timing Settings for 160x120 =====
  if (width == 160 && height == 120) {
    // Active area
    ov5647_write_reg(0x3800, 0x01);
    ov5647_write_reg(0x3801, 0x5c);
    ov5647_write_reg(0x3802, 0x01);
    ov5647_write_reg(0x3803, 0xb2);
    ov5647_write_reg(0x3804, 0x08);
    ov5647_write_reg(0x3805, 0xe3);
    ov5647_write_reg(0x3806, 0x05);
    ov5647_write_reg(0x3807, 0xf1);

    // Output size
    ov5647_write_reg(0x3808, 0x00);
    ov5647_write_reg(0x3809, 0xa0); // 160
    ov5647_write_reg(0x380a, 0x00);
    ov5647_write_reg(0x380b, 0x78); // 120

    // Total size (HTS/VTS)
    ov5647_write_reg(0x380c, 0x07);
    ov5647_write_reg(0x380d, 0x68); // 1896
    ov5647_write_reg(0x380e, 0x03);
    ov5647_write_reg(0x380f, 0xd8); // 984

    // Offset
    ov5647_write_reg(0x3810, 0x00);
    ov5647_write_reg(0x3811, 0x10);
    ov5647_write_reg(0x3812, 0x00);
    ov5647_write_reg(0x3813, 0x06);

    // Binning
    ov5647_write_reg(0x3814, 0x71); // X increment
    ov5647_write_reg(0x3815, 0x71); // Y increment
  }

  // ===== Timing Control =====
  ov5647_write_reg(0x3820, 0x00); // No vertical flip
  ov5647_write_reg(0x3821, 0x00); // No horizontal mirror

  // ===== AEC/AGC Settings =====
  ov5647_write_reg(0x3a02, 0x03);
  ov5647_write_reg(0x3a03, 0xd8);
  ov5647_write_reg(0x3a08, 0x01);
  ov5647_write_reg(0x3a09, 0x27);
  ov5647_write_reg(0x3a0a, 0x00);
  ov5647_write_reg(0x3a0b, 0xf6);
  ov5647_write_reg(0x3a0d, 0x04);
  ov5647_write_reg(0x3a0e, 0x03);
  ov5647_write_reg(0x3a14, 0x03);
  ov5647_write_reg(0x3a15, 0xd8);

  // ===== 50/60Hz Detection =====
  ov5647_write_reg(0x3c07, 0x08);

  vTaskDelay(pdMS_TO_TICKS(50));

  // ===== CRITICAL: Manual stream control OFF =====
  ov5647_write_reg(0x4202, 0x00); // Stream control: manual off

  vTaskDelay(pdMS_TO_TICKS(10));

  // ===== CRITICAL: Start Streaming =====
  if (!ov5647_write_reg(0x0100, 0x01)) {
    ESP_LOGE(TAG, "Failed to start streaming");
    return false;
  }

  // Wait for streaming to stabilize
  vTaskDelay(pdMS_TO_TICKS(300));

  // Verify
  uint8_t standby = 0, mipi_ctrl = 0, stream_ctrl = 0;
  ov5647_read_reg(0x0100, &standby);
  ov5647_read_reg(0x4800, &mipi_ctrl);
  ov5647_read_reg(0x4202, &stream_ctrl);
  ESP_LOGI(TAG, "Streaming: 0x%02X, MIPI: 0x%02X, Stream ctrl: 0x%02X", standby,
           mipi_ctrl, stream_ctrl);

  ESP_LOGI(TAG, "OV5647 sensor initialized and streaming");
  return true;
}

bool EiCameraESP32P4::init(uint16_t width, uint16_t height) {
  ei_device_snapshot_resolutions_t res = search_resolution(width, height);
  set_resolution(res);

  ESP_LOGI(TAG, "Initializing camera %dx%d", this->width, this->height);

  // 1. Create I2C master bus (NEW API)
  i2c_master_bus_config_t bus_config = {};
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.i2c_port = I2C_NUM_0;
  bus_config.scl_io_num = (gpio_num_t)SIOC_GPIO_NUM;
  bus_config.sda_io_num = (gpio_num_t)SIOD_GPIO_NUM;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "I2C master bus created");

  // 2. Add OV5647 device to bus (NEW API)
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = OV5647_I2C_ADDR;
  dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

  ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config,
                                  &ov5647_dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
    i2c_del_master_bus(i2c_bus_handle);
    i2c_bus_handle = NULL;
    return false;
  }

  ESP_LOGI(TAG, "OV5647 device added to I2C bus");

  // 3. Check OV5647 chip ID
  vTaskDelay(pdMS_TO_TICKS(50));
  uint8_t chip_id_h = 0, chip_id_l = 0;

  if (!ov5647_read_reg(OV5647_CHIP_ID_H, &chip_id_h) ||
      !ov5647_read_reg(OV5647_CHIP_ID_L, &chip_id_l)) {
    ESP_LOGE(TAG, "Cannot read OV5647 chip ID");
    camera_present = false;
    i2c_master_bus_rm_device(ov5647_dev_handle);
    i2c_del_master_bus(i2c_bus_handle);
    ov5647_dev_handle = NULL;
    i2c_bus_handle = NULL;
    return false;
  }

  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  if (chip_id != OV5647_CHIP_ID_VALUE) {
    ESP_LOGE(TAG, "Wrong chip ID: 0x%04X (expected 0x%04X)", chip_id,
             OV5647_CHIP_ID_VALUE);
    camera_present = false;
    i2c_master_bus_rm_device(ov5647_dev_handle);
    i2c_del_master_bus(i2c_bus_handle);
    ov5647_dev_handle = NULL;
    i2c_bus_handle = NULL;
    return false;
  }

  ESP_LOGI(TAG, "OV5647 detected, chip ID: 0x%04X", chip_id);
  camera_present = true;

  // 4. Initialize OV5647 sensor registers
  if (!ov5647_init_sensor(this->width, this->height)) {
    ESP_LOGE(TAG, "OV5647 sensor initialization failed");
    return false;
  }

  // 5. Configure CSI controller for OV5647
  esp_cam_ctlr_csi_config_t csi_config = {
      .ctlr_id = CSI_CTLR_ID,
      .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
      .h_res = this->width,
      .v_res = this->height,
      .data_lane_num = 2,
      .lane_bit_rate_mbps = 400,
      .input_data_color_type = CAM_CTLR_COLOR_RAW8,
      .output_data_color_type = CAM_CTLR_COLOR_RGB565, // ISP sẽ convert
      .queue_items = 1,
      .byte_swap_en = false,
      .bk_buffer_dis = false,
  };

  ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(ret));
    return false;
  }

  // ADD ISP PROCESSOR
  isp_proc_handle_t isp_proc = NULL;
  esp_isp_processor_cfg_t isp_config = {
      .clk_hz = 120 * 1000 * 1000, // 120 MHz ISP clock
      .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
      .input_data_color_type = ISP_COLOR_RAW8,
      .output_data_color_type = ISP_COLOR_RGB565, // Match CSI output
      .has_line_start_packet = false,
      .has_line_end_packet = false,
      .h_res = this->width,
      .v_res = this->height,
      .bayer_order = ISP_BAYER_BGGR, // OV5647 Bayer order
  };

  ret = esp_isp_new_processor(&isp_config, &isp_proc);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP processor init failed: %s", esp_err_to_name(ret));
    esp_cam_ctlr_del(cam_handle);
    return false;
  }

  // Configure ISP demosaic (RAW to RGB conversion)
  esp_isp_demosaic_config_t demosaic_config = {
      .grad_ratio =
          {
              .integer = 2,
              .decimal = 5,
          },
  };
  ret = esp_isp_demosaic_configure(isp_proc, &demosaic_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP demosaic config failed: %s", esp_err_to_name(ret));
  }

  // Enable ISP demosaic
  ret = esp_isp_demosaic_enable(isp_proc);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP demosaic enable failed: %s", esp_err_to_name(ret));
  }

  // Enable ISP processor
  ret = esp_isp_enable(isp_proc);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP enable failed: %s", esp_err_to_name(ret));
    esp_isp_del_processor(isp_proc);
    esp_cam_ctlr_del(cam_handle);
    return false;
  }

  ESP_LOGI(TAG, "ISP processor enabled");

  // 6. Register CSI callbacks (existing code)
  esp_cam_ctlr_evt_cbs_t cbs = {
      .on_get_new_trans = NULL,
      .on_trans_finished = on_trans_finished_cb,
  };

  ret = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(ret));
    esp_isp_disable(isp_proc);
    esp_isp_del_processor(isp_proc);
    esp_cam_ctlr_del(cam_handle);
    return false;
  }

  // 7. Enable and start camera controller
  ret = esp_cam_ctlr_enable(cam_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera enable failed: %s", esp_err_to_name(ret));
    return false;
  }

  ret = esp_cam_ctlr_start(cam_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera start failed: %s", esp_err_to_name(ret));
    return false;
  }

  // 8. Allocate frame buffer (RGB565: 2 bytes/pixel)
  if (CSI_OUTPUT_COLOR == CAM_CTLR_COLOR_RGB565) {
    frame_buffer_size = this->width * this->height * 2;
  } else {
    frame_buffer_size = this->width * this->height * 3;
  }

  frame_buffer = (uint8_t *)heap_caps_malloc(
      frame_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!frame_buffer) {
    ESP_LOGE(TAG, "Frame buffer allocation failed (%d bytes)",
             frame_buffer_size);
    esp_cam_ctlr_stop(cam_handle);
    esp_cam_ctlr_disable(cam_handle);
    esp_cam_ctlr_del(cam_handle);
    return false;
  }

  // 9. Camera warm-up and test capture
  ESP_LOGI(TAG, "Camera warming up...");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Test capture to verify setup
  esp_cam_ctlr_trans_t test_trans = {
      .buffer = frame_buffer,
      .buflen = frame_buffer_size,
  };

  ret = esp_cam_ctlr_receive(cam_handle, &test_trans, pdMS_TO_TICKS(10000));
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Camera init OK, test frame captured (%d bytes)",
             test_trans.received_size);
  } else {
    ESP_LOGW(TAG, "Test capture failed: %s, received %d bytes",
             esp_err_to_name(ret), test_trans.received_size);
  }
  ESP_LOGI(TAG, "CSI controller started, waiting for MIPI sync...");
  vTaskDelay(pdMS_TO_TICKS(500));

  return true;
}

bool EiCameraESP32P4::deinit() {
  ESP_LOGI(TAG, "Deinitializing camera");

  // Stop camera
  if (cam_handle) {
    esp_err_t err = esp_cam_ctlr_stop(cam_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Camera stop failed: %s", esp_err_to_name(err));
    }

    err = esp_cam_ctlr_disable(cam_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Camera disable failed: %s", esp_err_to_name(err));
    }

    err = esp_cam_ctlr_del(cam_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Camera delete failed: %s", esp_err_to_name(err));
      return false;
    }

    cam_handle = NULL;
  }

  // Free frame buffer
  if (frame_buffer) {
    heap_caps_free(frame_buffer);
    frame_buffer = NULL;
  }

  // Remove I2C device and delete bus (NEW API)
  if (ov5647_dev_handle) {
    i2c_master_bus_rm_device(ov5647_dev_handle);
    ov5647_dev_handle = NULL;
  }

  if (i2c_bus_handle) {
    i2c_del_master_bus(i2c_bus_handle);
    i2c_bus_handle = NULL;
  }

  camera_present = false;
  ESP_LOGI(TAG, "Camera deinitialized");
  return true;
}

bool EiCameraESP32P4::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image, uint32_t image_size) {
  if (!cam_handle || !frame_buffer) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  // Delay dài hơn để đảm bảo transaction trước đã được process
  vTaskDelay(pdMS_TO_TICKS(100));

  // Setup transaction
  s_trans.buffer = frame_buffer;
  s_trans.buflen = frame_buffer_size;
  s_trans.received_size = 0; // Reset received_size

  // Receive frame
  esp_err_t err =
      esp_cam_ctlr_receive(cam_handle, &s_trans, pdMS_TO_TICKS(3000));

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera capture failed: %s", esp_err_to_name(err));

    // Nếu timeout hoặc fail, đợi thêm rồi retry 1 lần
    if (err == ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout, waiting and retry once...");
      vTaskDelay(pdMS_TO_TICKS(500));

      s_trans.buffer = frame_buffer;
      s_trans.buflen = frame_buffer_size;
      s_trans.received_size = 0;

      err = esp_cam_ctlr_receive(cam_handle, &s_trans, pdMS_TO_TICKS(3000));
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Retry failed: %s", esp_err_to_name(err));
        return false;
      }
    } else {
      return false;
    }
  }

  // Verify received data
  if (s_trans.received_size == 0) {
    ESP_LOGE(TAG, "Received 0 bytes - sensor not streaming");
    return false;
  }

  ESP_LOGI(TAG, "Frame captured: %d bytes", s_trans.received_size);

  // Convert RGB565 to RGB888
  if (CSI_OUTPUT_COLOR == CAM_CTLR_COLOR_RGB565) {
    uint16_t *src = (uint16_t *)frame_buffer;
    uint8_t *dst = image;

    for (uint32_t i = 0; i < (width * height); i++) {
      uint16_t pixel = src[i];
      dst[i * 3 + 0] = ((pixel >> 11) & 0x1F) << 3; // R
      dst[i * 3 + 1] = ((pixel >> 5) & 0x3F) << 2;  // G
      dst[i * 3 + 2] = (pixel & 0x1F) << 3;         // B
    }
  } else {
    memcpy(image, frame_buffer, image_size);
  }

  return true;
}

bool EiCameraESP32P4::ei_camera_capture_jpeg(uint8_t **image,
                                             uint32_t *image_size) {
  // Allocate RGB buffer
  uint8_t *rgb_buffer = (uint8_t *)ei_malloc(width * height * 3);
  if (!rgb_buffer) {
    ESP_LOGE(TAG, "Failed to allocate RGB buffer");
    return false;
  }

  // Capture RGB888
  if (!ei_camera_capture_rgb888_packed_big_endian(rgb_buffer,
                                                  width * height * 3)) {
    ei_free(rgb_buffer);
    return false;
  }

  // TODO: Implement RGB to JPEG encoding using ESP JPEG encoder
  // For now, return RGB data (Edge Impulse daemon can handle RGB)
  ESP_LOGW(TAG, "JPEG encoding not implemented, returning RGB888 data");
  *image = rgb_buffer;
  *image_size = width * height * 3;

  return true;
}

bool EiCameraESP32P4::ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image,
                                               uint32_t jpeg_image_size,
                                               uint8_t *rgb888_image) {
  // TODO: Implement JPEG to RGB888 decoding using ESP JPEG decoder
  ESP_LOGE(TAG, "JPEG decoding not implemented");
  return false;
}

EiCamera *EiCamera::get_camera() {
  static EiCameraESP32P4 camera;
  return &camera;
}