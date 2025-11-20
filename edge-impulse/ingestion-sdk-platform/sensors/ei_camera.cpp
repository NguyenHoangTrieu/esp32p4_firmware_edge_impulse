#include "ei_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"

#include "cam_config.h"
#include "cam_sensor_init.h"

#include "driver/isp.h"
#include "esp_cache.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#include "hal/cache_hal.h"
#include "hal/cache_ll.h"

static const char *TAG = "EI_CAMERA";

static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static void *s_frame_buffer = NULL;
static void *s_take_frame_buffer = NULL;
static size_t s_frame_buffer_size = 0;
static esp_cam_ctlr_trans_t s_new_trans = {};
static bool s_is_camera_init = false;

ei_device_snapshot_resolutions_t EiCameraESP32P4::resolutions[] = {
    {.width = 800, .height = 640},
    {.width = 800, .height = 800},
    {.width = 800, .height = 1280},
};

EiCameraESP32P4::EiCameraESP32P4()
    : width(EI_CLASSIFIER_INPUT_WIDTH), height(EI_CLASSIFIER_INPUT_HEIGHT),
      output_width(EI_CLASSIFIER_INPUT_WIDTH),
      output_height(EI_CLASSIFIER_INPUT_HEIGHT), camera_present(true) {}

static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data) {
  memcpy(s_take_frame_buffer, trans->buffer, trans->buflen);
  memset(trans->buffer, 0xFF, trans->buflen);
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
  output_width = res.width;
  output_height = res.height;
  return true;
}

bool EiCameraESP32P4::init(uint16_t w, uint16_t h) {
  if (s_is_camera_init) {
    ESP_LOGW(TAG, "Camera already initialized");
    return true;
  }
  width = w;
  height = h;
  output_width = w;
  output_height = h;

  esp_err_t ret = ESP_FAIL;
  // mipi ldo
  ESP_LOGI(TAG, "Init camera %dx%d", width, height);
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
      .chan_id = 3,
      .voltage_mv = 2500,
  };
  ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

  //--------Camera Sensor and SCCB Init-----------//
  i2c_master_bus_handle_t i2c_bus_handle = NULL;
  cam_sensor_init(I2C_NUM_0, &i2c_bus_handle);

  //---------------CSI Init------------------//
  esp_cam_ctlr_csi_config_t csi_config = {
      .ctlr_id = 0,
      .h_res = EI_CLASSIFIER_INPUT_WIDTH,
      .v_res = EI_CLASSIFIER_INPUT_HEIGHT,
      .data_lane_num = 2,
      .lane_bit_rate_mbps = CAM_MIPI_CSI_LANE_BITRATE_MBPS,
      .input_data_color_type = CAM_CTLR_COLOR_RAW8,
      .output_data_color_type = CAM_CTLR_COLOR_RGB888,
      .queue_items = 1,
      .byte_swap_en = false,
  };
  ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "csi init fail[%d]", ret);
    return false;
  }
  //---------------Necessary variable config------------------//
  s_frame_buffer_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT *
                        CAM_RGB888_BITS_PER_PIXEL / 8;
  s_frame_buffer = esp_cam_ctlr_alloc_buffer(
      s_cam_handle, s_frame_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
  ESP_LOGD(TAG,
           "EI_CLASSIFIER_INPUT_WIDTH: %d, EI_CLASSIFIER_INPUT_HEIGHT: %d, "
           "bits per pixel: %d",
           EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 8);
  ESP_LOGD(TAG, "s_frame_buffer_size: %zu", s_frame_buffer_size);
  ESP_LOGD(TAG, "s_frame_buffer: %p", s_frame_buffer);
  s_take_frame_buffer = heap_caps_calloc(1, s_frame_buffer_size,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_frame_buffer || !s_take_frame_buffer) {
    ESP_LOGE(TAG, "frame buffer malloc fail");
    return false;
  }
  s_new_trans.buffer = s_frame_buffer;
  s_new_trans.buflen = s_frame_buffer_size;

  esp_cam_ctlr_evt_cbs_t cbs = {
      .on_get_new_trans = NULL,
      .on_trans_finished = s_camera_get_finished_trans,
  };
  if (esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, &s_new_trans) !=
      ESP_OK) {
    ESP_LOGE(TAG, "ops register fail");
    return false;
  }

  ESP_ERROR_CHECK(esp_cam_ctlr_enable(s_cam_handle));

  //---------------ISP Init------------------//
  esp_isp_processor_cfg_t s_isp_config = {
      .clk_hz = 80 * 1000 * 1000,
      .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
      .input_data_color_type = ISP_COLOR_RAW8,
      .output_data_color_type = ISP_COLOR_RGB888,
      .has_line_start_packet = false,
      .has_line_end_packet = false,
      .h_res = EI_CLASSIFIER_INPUT_WIDTH,
      .v_res = EI_CLASSIFIER_INPUT_HEIGHT,
  };
  ESP_ERROR_CHECK(esp_isp_new_processor(&s_isp_config, &s_isp_proc));
  ESP_ERROR_CHECK(esp_isp_enable(s_isp_proc));

  //---------------DPI Reset------------------//
  // example_dpi_panel_reset(mipi_dpi_panel);

  // init to all white
  memset(s_frame_buffer, 0xFF, s_frame_buffer_size);
  esp_cache_msync((void *)s_frame_buffer, s_frame_buffer_size,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  if (esp_cam_ctlr_start(s_cam_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Driver start fail");
    return false;
  }
  camera_present = true;
  s_is_camera_init = true;
  return true;
}

bool EiCameraESP32P4::deinit() {
  ESP_LOGI(TAG, "Deinit camera");
  if (s_cam_handle) {
    esp_cam_ctlr_stop(s_cam_handle);
    esp_cam_ctlr_disable(s_cam_handle);
    esp_cam_ctlr_del(s_cam_handle);
  }
  if (s_isp_proc) {
    esp_isp_disable(s_isp_proc);
    esp_isp_del_processor(s_isp_proc);
  }
  if (ldo_mipi_phy)
    esp_ldo_release_channel(ldo_mipi_phy);
  if (s_frame_buffer)
    heap_caps_free(s_frame_buffer);

  camera_present = false;
  return true;
}

bool EiCameraESP32P4::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image, uint32_t image_size) {
  ESP_LOGI(TAG, "Image size requested: %d", image_size);
  if (!s_is_camera_init) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  uint32_t required = width * height * 3;
  if (image_size < required)
    return false;
  esp_err_t ret =
      esp_cam_ctlr_receive(s_cam_handle, &s_new_trans, pdMS_TO_TICKS(1000));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Receive frame fail[%d]", ret);
  }
  uint8_t *src = (uint8_t *)s_take_frame_buffer;
  ESP_LOGI(TAG, "First 10 bytes: ");
  for (int i = 0; i < 10; i++) {
    esp_rom_printf("%02x ", src[i]);
  }
  esp_rom_printf("\r\n");
  image = (uint8_t *)s_take_frame_buffer;
  return true;
}

bool EiCameraESP32P4::ei_camera_capture_jpeg(uint8_t **image,
                                             uint32_t *image_size) {
  uint32_t size = width * height * 3;
  if (ei_camera_capture_rgb888_packed_big_endian(*image, size)) {
    *image_size = size;
    return true;
  }

  heap_caps_free(*image);
  return false;
}

bool EiCameraESP32P4::ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image,
                                               uint32_t jpeg_image_size,
                                               uint8_t *rgb888_image) {
  memcpy(rgb888_image, jpeg_image, jpeg_image_size);
  return true;
}

EiCamera *EiCamera::get_camera() {
  static EiCameraESP32P4 camera;
  return &camera;
}

// AT+RUNIMPULSE
// AT+SNAPSHOT=800,1280,n