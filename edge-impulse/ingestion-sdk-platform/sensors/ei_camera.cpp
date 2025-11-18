/* Edge Impulse Camera Driver for ESP32-P4 with OV5647
 * Based on ESP-IDF mipi_isp_dsi example
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

// ISP headers
#include "driver/isp.h"
#include "driver/isp_core.h"

// JPEG encoder/decoder
#include "driver/jpeg_encode.h"
#include "driver/jpeg_decode.h"

static const char *TAG = "OV5647";

// OV5647 I2C definitions
#define OV5647_I2C_ADDR 0x36
#define OV5647_CHIP_ID_H 0x300A
#define OV5647_CHIP_ID_L 0x300B
#define OV5647_CHIP_ID_VALUE 0x5647
#define OV5647_SW_RESET 0x0103
#define OV5647_SW_STANDBY 0x0100

// I2C configuration
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

// Static handles
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t ov5647_dev_handle = NULL;
static isp_proc_handle_t isp_proc = NULL;
static jpeg_encoder_handle_t jpeg_enc_handle = NULL;
static jpeg_decoder_handle_t jpeg_dec_handle = NULL;

// Frame buffer management (double buffer like example)
#define FRAME_BUFFER_NUM 2
static uint8_t *frame_buffers[FRAME_BUFFER_NUM];
static size_t frame_buffer_size = 0;

/* Resolution configurations */
ei_device_snapshot_resolutions_t EiCameraESP32P4::resolutions[] = {
    {.width = 160, .height = 120},
    {.width = 320, .height = 240},
    {.width = 640, .height = 480},
    {.width = 800, .height = 600}
};

EiCameraESP32P4::EiCameraESP32P4()
    : width(640), height(480), output_width(640), output_height(480),
      camera_present(false), cam_handle(NULL), frame_buffer(NULL),
      frame_buffer_size(0) {}

// ===== ISP and CSI Callbacks (From Example) =====
static bool IRAM_ATTR on_get_new_trans_callback(esp_cam_ctlr_handle_t handle,
                                                 esp_cam_ctlr_trans_t *trans,
                                                 void *user_data)
{
    static uint8_t buf_index = 0;
    
    if (trans && frame_buffers[buf_index]) {
        trans->buffer = frame_buffers[buf_index];
        trans->buflen = frame_buffer_size;
        buf_index = (buf_index + 1) % FRAME_BUFFER_NUM;
    }
    
    return false;
}

static bool IRAM_ATTR on_trans_finished_callback(esp_cam_ctlr_handle_t handle,
                                                  esp_cam_ctlr_trans_t *trans,
                                                  void *user_data)
{
    if (trans && trans->received_size > 0) {
        ESP_EARLY_LOGD(TAG, "Frame received: %d bytes", trans->received_size);
    }
    return false;
}

bool EiCameraESP32P4::is_camera_present(void) { return camera_present; }

ei_device_snapshot_resolutions_t EiCameraESP32P4::get_min_resolution(void)
{
    return resolutions[0];
}

void EiCameraESP32P4::get_resolutions(ei_device_snapshot_resolutions_t **res,
                                      uint8_t *res_num)
{
    *res = &EiCameraESP32P4::resolutions[0];
    *res_num = sizeof(EiCameraESP32P4::resolutions) /
               sizeof(ei_device_snapshot_resolutions_t);
}

bool EiCameraESP32P4::set_resolution(const ei_device_snapshot_resolutions_t res)
{
    width = res.width;
    height = res.height;
    ESP_LOGI(TAG, "Resolution set to %dx%d", width, height);
    return true;
}

/* I2C Helper Functions */
static bool ov5647_read_reg(uint16_t reg_addr, uint8_t *data)
{
    if (!ov5647_dev_handle) return false;

    uint8_t reg_buf[2] = {(uint8_t)((reg_addr >> 8) & 0xFF),
                          (uint8_t)(reg_addr & 0xFF)};
    
    esp_err_t ret = i2c_master_transmit_receive(ov5647_dev_handle, reg_buf,
                                                sizeof(reg_buf), data, 1,
                                                I2C_MASTER_TIMEOUT_MS);
    return (ret == ESP_OK);
}

static bool ov5647_write_reg(uint16_t reg_addr, uint8_t data)
{
    if (!ov5647_dev_handle) return false;

    uint8_t buf[3] = {(uint8_t)((reg_addr >> 8) & 0xFF),
                      (uint8_t)(reg_addr & 0xFF), data};
    
    esp_err_t ret = i2c_master_transmit(ov5647_dev_handle, buf, sizeof(buf),
                                        I2C_MASTER_TIMEOUT_MS);
    return (ret == ESP_OK);
}

static bool ov5647_init_sensor(uint16_t width, uint16_t height)
{
    ESP_LOGI(TAG, "Initializing OV5647 sensor for %dx%d", width, height);

    // Software reset
    ov5647_write_reg(0x0103, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enter standby
    ov5647_write_reg(0x0100, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // PLL and Clock
    ov5647_write_reg(0x3034, 0x1a);
    ov5647_write_reg(0x3035, 0x21);
    ov5647_write_reg(0x3036, 0x69);
    ov5647_write_reg(0x303c, 0x11);
    ov5647_write_reg(0x3106, 0xf5);

    // MIPI Control
    ov5647_write_reg(0x4800, 0x24);
    ov5647_write_reg(0x4837, 0x16);
    ov5647_write_reg(0x300e, 0x45);
    ov5647_write_reg(0x4814, 0x2b);
    ov5647_write_reg(0x300a, 0xff);
    ov5647_write_reg(0x300b, 0xff);

    // System Control
    ov5647_write_reg(0x3000, 0x00);
    ov5647_write_reg(0x3001, 0x00);
    ov5647_write_reg(0x3002, 0x00);
    ov5647_write_reg(0x3016, 0x08);
    ov5647_write_reg(0x3017, 0xe0);
    ov5647_write_reg(0x3018, 0x44);
    ov5647_write_reg(0x301c, 0xf8);
    ov5647_write_reg(0x301d, 0xf0);

    // Format - RAW8
    ov5647_write_reg(0x4300, 0x60);

    // ISP Control
    ov5647_write_reg(0x5000, 0x06);
    ov5647_write_reg(0x5001, 0x01);
    ov5647_write_reg(0x5002, 0x00);

    // BLC
    ov5647_write_reg(0x4000, 0x09);
    ov5647_write_reg(0x4001, 0x02);
    ov5647_write_reg(0x4002, 0xc5);

    // Analog Control
    ov5647_write_reg(0x3620, 0x52);
    ov5647_write_reg(0x3621, 0xe0);
    ov5647_write_reg(0x3622, 0x01);
    ov5647_write_reg(0x3630, 0x2e);
    ov5647_write_reg(0x3632, 0xe2);
    ov5647_write_reg(0x3633, 0x23);
    ov5647_write_reg(0x3634, 0x44);

    // Timing for resolution
    if (width == 640 && height == 480) {
        ov5647_write_reg(0x3800, 0x00); ov5647_write_reg(0x3801, 0x00);
        ov5647_write_reg(0x3802, 0x00); ov5647_write_reg(0x3803, 0x00);
        ov5647_write_reg(0x3804, 0x0a); ov5647_write_reg(0x3805, 0x3f);
        ov5647_write_reg(0x3806, 0x07); ov5647_write_reg(0x3807, 0xa3);
        ov5647_write_reg(0x3808, 0x02); ov5647_write_reg(0x3809, 0x80);
        ov5647_write_reg(0x380a, 0x01); ov5647_write_reg(0x380b, 0xe0);
        ov5647_write_reg(0x380c, 0x07); ov5647_write_reg(0x380d, 0x68);
        ov5647_write_reg(0x380e, 0x03); ov5647_write_reg(0x380f, 0xd8);
        ov5647_write_reg(0x3810, 0x00); ov5647_write_reg(0x3811, 0x10);
        ov5647_write_reg(0x3812, 0x00); ov5647_write_reg(0x3813, 0x06);
        ov5647_write_reg(0x3814, 0x31);
        ov5647_write_reg(0x3815, 0x31);
    } else if (width == 160 && height == 120) {
        ov5647_write_reg(0x3800, 0x01); ov5647_write_reg(0x3801, 0x5c);
        ov5647_write_reg(0x3802, 0x01); ov5647_write_reg(0x3803, 0xb2);
        ov5647_write_reg(0x3804, 0x08); ov5647_write_reg(0x3805, 0xe3);
        ov5647_write_reg(0x3806, 0x05); ov5647_write_reg(0x3807, 0xf1);
        ov5647_write_reg(0x3808, 0x00); ov5647_write_reg(0x3809, 0xa0);
        ov5647_write_reg(0x380a, 0x00); ov5647_write_reg(0x380b, 0x78);
        ov5647_write_reg(0x380c, 0x07); ov5647_write_reg(0x380d, 0x68);
        ov5647_write_reg(0x380e, 0x03); ov5647_write_reg(0x380f, 0xd8);
        ov5647_write_reg(0x3810, 0x00); ov5647_write_reg(0x3811, 0x10);
        ov5647_write_reg(0x3812, 0x00); ov5647_write_reg(0x3813, 0x06);
        ov5647_write_reg(0x3814, 0x71);
        ov5647_write_reg(0x3815, 0x71);
    }

    // Timing control
    ov5647_write_reg(0x3820, 0x00);
    ov5647_write_reg(0x3821, 0x00);

    vTaskDelay(pdMS_TO_TICKS(50));

    // Stream control
    ov5647_write_reg(0x4202, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Start streaming
    ov5647_write_reg(0x0100, 0x01);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "OV5647 streaming started");
    return true;
}

bool EiCameraESP32P4::init(uint16_t width, uint16_t height)
{
    ei_device_snapshot_resolutions_t res = search_resolution(width, height);
    set_resolution(res);

    ESP_LOGI(TAG, "Initializing camera %dx%d", this->width, this->height);

    // 1. I2C Setup
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = (gpio_num_t)SIOC_GPIO_NUM;
    bus_config.sda_io_num = (gpio_num_t)SIOD_GPIO_NUM;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return false;
    }

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = OV5647_I2C_ADDR;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &ov5647_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed");
        return false;
    }

    // 2. Check Chip ID
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t chip_id_h = 0, chip_id_l = 0;

    if (!ov5647_read_reg(OV5647_CHIP_ID_H, &chip_id_h) ||
        !ov5647_read_reg(OV5647_CHIP_ID_L, &chip_id_l)) {
        ESP_LOGE(TAG, "Cannot read chip ID");
        camera_present = false;
        return false;
    }

    uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
    if (chip_id != OV5647_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Wrong chip ID: 0x%04X", chip_id);
        camera_present = false;
        return false;
    }

    ESP_LOGI(TAG, "OV5647 detected, chip ID: 0x%04X", chip_id);
    camera_present = true;

    // 3. Init OV5647
    if (!ov5647_init_sensor(this->width, this->height)) {
        ESP_LOGE(TAG, "Sensor init failed");
        return false;
    }

    // 4. ===== ISP Setup (REQUIRED for MIPI CSI per example) =====
    esp_isp_processor_cfg_t isp_config = {};
    isp_config.clk_hz = 80 * 1000 * 1000;
    isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
    isp_config.input_data_color_type = ISP_COLOR_RAW8;
    isp_config.output_data_color_type = ISP_COLOR_RGB565;
    isp_config.h_res = this->width;
    isp_config.v_res = this->height;
    isp_config.has_line_start_packet = false;
    isp_config.has_line_end_packet = false;

    ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISP init failed");
        return false;
    }

    ret = esp_isp_enable(isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISP enable failed");
        return false;
    }

    ESP_LOGI(TAG, "ISP enabled");

    // 5. ===== CSI Setup (After ISP like example) =====
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = CSI_CTLR_ID,
        .clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT,
        .h_res = this->width,
        .v_res = this->height,
        .data_lane_num = 2,
        .lane_bit_rate_mbps = 400,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .queue_items = 1,
        .byte_swap_en = false,
        .bk_buffer_dis = false,
    };

    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI init failed");
        return false;
    }

    // 6. Allocate Double Buffers (Like Example)
    frame_buffer_size = this->width * this->height * 2;  // RGB565
    
    for (int i = 0; i < FRAME_BUFFER_NUM; i++) {
        frame_buffers[i] = (uint8_t *)heap_caps_aligned_calloc(64, 1, frame_buffer_size,
                                                                MALLOC_CAP_SPIRAM);
        if (!frame_buffers[i]) {
            ESP_LOGE(TAG, "Buffer %d allocation failed", i);
            return false;
        }
    }

    // Use first buffer as primary
    frame_buffer = frame_buffers[0];
    this->frame_buffer_size = frame_buffer_size;

    ESP_LOGI(TAG, "Allocated %d buffers of %d bytes each", FRAME_BUFFER_NUM, frame_buffer_size);

    // 7. Register Callbacks (Like Example - Use Double Buffer)
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans_callback,
        .on_trans_finished = on_trans_finished_callback,
    };

    ret = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Callback register failed");
        return false;
    }

    // 8. Enable and Start (Like Example)
    ret = esp_cam_ctlr_enable(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI enable failed");
        return false;
    }

    ret = esp_cam_ctlr_start(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI start failed");
        return false;
    }

    // 9. JPEG Init (Optional)
    jpeg_encode_engine_cfg_t encode_eng_cfg = {};
    encode_eng_cfg.timeout_ms = 40;
    encode_eng_cfg.intr_priority = 0;

    jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_enc_handle);

    jpeg_decode_engine_cfg_t decode_eng_cfg = {};
    decode_eng_cfg.timeout_ms = 40;
    decode_eng_cfg.intr_priority = 0;

    jpeg_new_decoder_engine(&decode_eng_cfg, &jpeg_dec_handle);

    // 10. Warm-up
    ESP_LOGI(TAG, "Camera warming up...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Camera init complete");
    return true;
}

bool EiCameraESP32P4::deinit()
{
    ESP_LOGI(TAG, "Deinitializing camera");

    if (cam_handle) {
        esp_cam_ctlr_stop(cam_handle);
        esp_cam_ctlr_disable(cam_handle);
        esp_cam_ctlr_del(cam_handle);
        cam_handle = NULL;
    }

    if (isp_proc) {
        esp_isp_disable(isp_proc);
        esp_isp_del_processor(isp_proc);
        isp_proc = NULL;
    }

    for (int i = 0; i < FRAME_BUFFER_NUM; i++) {
        if (frame_buffers[i]) {
            heap_caps_free(frame_buffers[i]);
            frame_buffers[i] = NULL;
        }
    }

    if (jpeg_enc_handle) {
        jpeg_del_encoder_engine(jpeg_enc_handle);
        jpeg_enc_handle = NULL;
    }

    if (jpeg_dec_handle) {
        jpeg_del_decoder_engine(jpeg_dec_handle);
        jpeg_dec_handle = NULL;
    }

    if (ov5647_dev_handle) {
        i2c_master_bus_rm_device(ov5647_dev_handle);
        ov5647_dev_handle = NULL;
    }

    if (i2c_bus_handle) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }

    camera_present = false;
    return true;
}

bool EiCameraESP32P4::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image, uint32_t image_size)
{
    if (!cam_handle || !frame_buffer) {
        ESP_LOGE(TAG, "Camera not initialized");
        return false;
    }

    // Wait a bit for frame
    vTaskDelay(pdMS_TO_TICKS(100));

    // Frame is automatically captured to frame_buffers via callbacks
    // Just convert RGB565 to RGB888
    uint16_t *src = (uint16_t *)frame_buffers[0];  // Use buffer 0
    uint8_t *dst = image;

    for (uint32_t i = 0; i < (width * height); i++) {
        uint16_t pixel = src[i];
        dst[i * 3 + 0] = ((pixel >> 11) & 0x1F) << 3;  // R
        dst[i * 3 + 1] = ((pixel >> 5) & 0x3F) << 2;   // G
        dst[i * 3 + 2] = (pixel & 0x1F) << 3;          // B
    }

    return true;
}

bool EiCameraESP32P4::ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size)
{
    // ===== COMPLETE FIX: Skip JPEG entirely, return RGB888 =====
    ESP_LOGI(TAG, "Capturing RGB888 (no JPEG encoding)");
    
    // Allocate RGB888 buffer
    uint8_t *rgb_buffer = (uint8_t *)ei_malloc(width * height * 3);
    if (!rgb_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return false;
    }

    // Capture RGB888 directly
    if (!ei_camera_capture_rgb888_packed_big_endian(rgb_buffer, width * height * 3)) {
        ESP_LOGE(TAG, "RGB888 capture failed");
        ei_free(rgb_buffer);
        return false;
    }

    *image = rgb_buffer;
    *image_size = width * height * 3;
    
    ESP_LOGI(TAG, "RGB888 captured: %d bytes", *image_size);
    return true;
}

bool EiCameraESP32P4::ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image,
                                               uint32_t jpeg_image_size,
                                               uint8_t *rgb888_image)
{
    // Not needed if we return RGB888 directly
    ESP_LOGW(TAG, "JPEG decode skipped - using RGB888 directly");
    return false;
}

EiCamera *EiCamera::get_camera()
{
    static EiCameraESP32P4 camera;
    return &camera;
}
