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
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EI_CAMERA
#define EI_CAMERA

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_camera_interface.h"

#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_types.h"
#include "esp_cam_ctlr_csi.h"

// Define camera model for ESP32-P4 with MIPI CSI
#define CAMERA_MODEL_ESP32P4_MIPI_CSI

#if defined(CAMERA_MODEL_ESP32P4_MIPI_CSI)
// ESP32-P4 MIPI CSI Configuration
// No GPIO pin definitions needed for CSI (uses dedicated MIPI lanes)
#define CSI_CTLR_ID 0                    // CSI controller ID
#define CSI_DATA_LANE_NUM 2              // Number of MIPI data lanes (2 or 4)
#define CSI_LANE_BITRATE_MBPS 1000       // Lane bit rate in Mbps (adjust based on sensor)
#define CSI_INPUT_COLOR CAM_CTLR_COLOR_RAW8    // Input format from sensor
#define CSI_OUTPUT_COLOR CAM_CTLR_COLOR_RGB565  // Output format after ISP
#define CSI_QUEUE_ITEMS 8                // Number of transaction queue items

// I2C pins for sensor control (SCCB/I2C interface)
#define SIOD_GPIO_NUM 7                  // I2C SDA
#define SIOC_GPIO_NUM 8                  // I2C SCL

// Optional: LED pins if available
#define LED_PIN -1                       // Status LED (if available)
#define LAMP_PIN -1                      // Flash lamp (if available)

#else
#error "Camera model not selected or unsupported for ESP32-P4"
#endif

class EiCameraESP32P4 : public EiCamera {
private:
    static ei_device_snapshot_resolutions_t resolutions[];
    uint32_t width;
    uint32_t height;
    uint32_t output_width;
    uint32_t output_height;
    bool camera_present;
public:
    EiCameraESP32P4();
    bool init(uint16_t width, uint16_t height);
    bool deinit();
    bool ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size);
    bool ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size);
    bool ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image, uint32_t jpeg_image_size, 
                                   uint8_t *rgb88_image);
    bool set_resolution(const ei_device_snapshot_resolutions_t res);
    ei_device_snapshot_resolutions_t get_min_resolution(void);
    bool is_camera_present(void);
    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num);
};

#endif /* EI_CAMERA */