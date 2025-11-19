#include "ei_camera.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"

extern "C" {
#include "example_config.h"
#include "example_sensor_init.h"
}

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

static const char *TAG = "EI_CAMERA";

static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static example_sensor_handle_t sensor_handle = {
    .sccb_handle = NULL,
    .i2c_bus_handle = NULL,
};
static esp_cam_ctlr_handle_t cam_handle = NULL;
static isp_proc_handle_t isp_proc = NULL;
static void *s_frame_buffer = NULL;
static size_t s_frame_buffer_size = 0;
static esp_cam_ctlr_trans_t s_new_trans = {};

ei_device_snapshot_resolutions_t EiCameraESP32P4::resolutions[] = {
    {.width = 800, .height = 640},
    {.width = 800, .height = 800},
    {.width = 800, .height = 1280},
};

EiCameraESP32P4::EiCameraESP32P4()
    : width(800), height(640), output_width(800), output_height(640),
      camera_present(false), cam_handle(NULL), frame_buffer(NULL),
      frame_buffer_size(0) {}

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle,
                                esp_cam_ctlr_trans_t *trans, void *user_data) {
  esp_cam_ctlr_trans_t new_trans = *(esp_cam_ctlr_trans_t *)user_data;
  trans->buffer = new_trans.buffer;
  trans->buflen = new_trans.buflen;
  return false;
}

static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans,
                                        void *user_data) {
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
  esp_err_t ret = ESP_FAIL;

  width = w;
  height = h;
  output_width = w;
  output_height = h;

  ESP_LOGI(TAG, "Init camera %dx%d", width, height);

  esp_ldo_channel_config_t ldo_cfg = {
      .chan_id = 3,
      .voltage_mv = 2500,
  };
  ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy));

  s_frame_buffer_size = width * height * 2;
  s_frame_buffer = (uint8_t *)calloc(1, s_frame_buffer_size);
  if (!s_frame_buffer) {
    ESP_LOGE(TAG, "Buffer calloc fail");
    return false;
  }
  ESP_LOGI(TAG, "frame_buffer_size: %zu", s_frame_buffer_size);
  ESP_LOGI(TAG, "frame_buffer: %p", s_frame_buffer);

  s_new_trans.buffer = s_frame_buffer;
  s_new_trans.buflen = s_frame_buffer_size;

  ESP_LOGI(TAG, "Buffer allocated: %d bytes", s_frame_buffer_size);

  example_sensor_config_t cam_sensor_config = {
      .i2c_port_num = I2C_NUM_0,
      .i2c_sda_io_num = SIOD_GPIO_NUM,
      .i2c_scl_io_num = SIOC_GPIO_NUM,
      .port = ESP_CAM_SENSOR_MIPI_CSI,
      .format_name = "MIPI_2lane_24Minput_RAW8_800x640_50fps",
  };
  example_sensor_init(&cam_sensor_config, &sensor_handle);

  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = width;
  csi_config.v_res = height;
  csi_config.lane_bit_rate_mbps = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 1;
  ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI init fail");
    return false;
  }

  esp_cam_ctlr_evt_cbs_t cbs = {
      .on_get_new_trans = s_camera_get_new_vb,
      .on_trans_finished = s_camera_get_finished_trans,
  };
  if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &s_new_trans) !=
      ESP_OK) {
    ESP_LOGE(TAG, "Callback register fail");
    return false;
  }

  ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

  // ISP processor init
  //   esp_isp_processor_cfg_t isp_config = {
  //       .clk_hz = 80 * 1000 * 1000,
  //       .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
  //       .input_data_color_type = ISP_COLOR_RAW8,
  //       .output_data_color_type = ISP_COLOR_RAW8,
  //       .has_line_start_packet = false,
  //       .has_line_end_packet = false,
  //       .h_res = width,
  //       .v_res = height,
  //   };
  //   ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
  //   ESP_ERROR_CHECK(esp_isp_enable(isp_proc));
  memset(s_frame_buffer, 0xFF, s_frame_buffer_size);
  esp_cache_msync((void *)s_frame_buffer, s_frame_buffer_size,
                  ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  if (esp_cam_ctlr_start(cam_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Start fail");
    return false;
  }

  camera_present = true;
  frame_buffer = (uint8_t *)s_frame_buffer;
  frame_buffer_size = s_frame_buffer_size;

  return true;
}

bool EiCameraESP32P4::deinit() {
  ESP_LOGI(TAG, "Deinit camera");
  if (cam_handle) {
    esp_cam_ctlr_stop(cam_handle);
    esp_cam_ctlr_disable(cam_handle);
    esp_cam_ctlr_del(cam_handle);
  }
  if (isp_proc) {
    esp_isp_disable(isp_proc);
    esp_isp_del_processor(isp_proc);
  }

  example_sensor_deinit(sensor_handle);

  if (ldo_mipi_phy)
    esp_ldo_release_channel(ldo_mipi_phy);
  if (s_frame_buffer)
    heap_caps_free(s_frame_buffer);

  camera_present = false;
  return true;
}

bool EiCameraESP32P4::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image, uint32_t image_size) {
  if (!camera_present)
    return false;

  uint32_t required = width * height * 3;
  if (image_size < required)
    return false;

  if (esp_cam_ctlr_receive(cam_handle, &s_new_trans, ESP_CAM_CTLR_MAX_DELAY) !=
      ESP_OK) {
    return false;
  }

  for (int i = 0; i < 100; i++) {
    ESP_LOGI(TAG, "First 100 bytes of frame buffer: %02X", ((uint8_t *)s_frame_buffer)[i]);
  }

  if (s_new_trans.received_size == 0) {
    ESP_LOGE(TAG, "Received size is 0");
    return false;
  }

  uint16_t *src = (uint16_t *)s_frame_buffer;
  for (uint32_t i = 0; i < width * height; i++) {
    uint16_t pixel = src[i];
    uint8_t r = (pixel >> 11) & 0x1F;
    uint8_t g = (pixel >> 5) & 0x3F;
    uint8_t b = pixel & 0x1F;

    image[i * 3 + 0] = (r << 3) | (r >> 2);
    image[i * 3 + 1] = (g << 2) | (g >> 4);
    image[i * 3 + 2] = (b << 3) | (b >> 2);
  }

  return true;
}

bool EiCameraESP32P4::ei_camera_capture_jpeg(uint8_t **image,
                                             uint32_t *image_size) {
  uint32_t size = width * height * 3;
  *image = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  if (!*image)
    return false;

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
