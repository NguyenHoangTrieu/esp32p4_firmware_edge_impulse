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
#include<stdint.h>
#include <stdlib.h>

// ESP-IDF 6.0 ADC headers
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "ei_analogsensor.h"
#include "ei_device_espressif_esp32.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"

    /* Constant defines --------------------------------------------------------
     */
static const char *TAG = "AnalogSensor";
static float analog_data[ANALOG_AXIS_SAMPLED];

// ADC handles for ESP-IDF 6.0
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// ADC configuration per target
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CHAN ADC_CHANNEL_6 // GPIO34
#define ADC_EXAMPLE_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_12
#define ADC_ATTEN_LEVEL ADC_ATTEN_DB_12 // 0-3.3V range
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CHAN ADC_CHANNEL_6 // GPIO7
#define ADC_EXAMPLE_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_13
#define ADC_ATTEN_LEVEL ADC_ATTEN_DB_12
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CHAN ADC_CHANNEL_6 // GPIO7
#define ADC_EXAMPLE_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_12
#define ADC_ATTEN_LEVEL ADC_ATTEN_DB_12
#elif CONFIG_IDF_TARGET_ESP32P4
#define ADC_EXAMPLE_CHAN ADC_CHANNEL_0 // Adjust for ESP32-P4
#define ADC_EXAMPLE_UNIT ADC_UNIT_1
#define ADC_BITWIDTH ADC_BITWIDTH_12
#define ADC_ATTEN_LEVEL ADC_ATTEN_DB_12
#else
#error "Unsupported target for ADC"
#endif

bool ei_analog_sensor_init(void) {
  esp_err_t ret;

  // 1. Initialize ADC unit (oneshot mode)
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_EXAMPLE_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ret = adc_oneshot_new_unit(&init_config, &adc_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
    return false;
  }

  // 2. Configure ADC channel
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_LEVEL,
      .bitwidth = ADC_BITWIDTH,
  };

  ret = adc_oneshot_config_channel(adc_handle, ADC_EXAMPLE_CHAN, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
    adc_oneshot_del_unit(adc_handle);
    return false;
  }

  // 3. Calibration (recommended for accurate voltage reading)
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_EXAMPLE_UNIT,
      .atten = ADC_ATTEN_LEVEL,
      .bitwidth = ADC_BITWIDTH,
  };
  ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "ADC calibration success (Line Fitting)");
  } else {
    ESP_LOGW(TAG, "ADC calibration failed, using raw values");
    adc_cali_handle = NULL;
  }
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_EXAMPLE_UNIT,
      .atten = ADC_ATTEN_LEVEL,
      .bitwidth = ADC_BITWIDTH,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "ADC calibration success (Curve Fitting)");
  } else {
    ESP_LOGW(TAG, "ADC calibration failed, using raw values");
    adc_cali_handle = NULL;
  }
#else
  ESP_LOGW(TAG, "No calibration scheme available, using raw values");
  adc_cali_handle = NULL;
#endif

  // Register sensor with Edge Impulse fusion
  ei_add_sensor_to_fusion_list(analog_sensor);

  ESP_LOGI(TAG, "Analog sensor initialized successfully");
  return true;
}

float *ei_fusion_analog_sensor_read_data(int n_samples) {
  int raw_value = 0;
  int voltage_mv = 0;

  // Read ADC raw value
  esp_err_t ret = adc_oneshot_read(adc_handle, ADC_EXAMPLE_CHAN, &raw_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
    analog_data[0] = 0.0f;
    return analog_data;
  }

  // Convert to voltage if calibration is available
  if (adc_cali_handle != NULL) {
    ret = adc_cali_raw_to_voltage(adc_cali_handle, raw_value, &voltage_mv);
    if (ret == ESP_OK) {
      // Store voltage in volts
      analog_data[0] = (float)voltage_mv / 1000.0f;
    } else {
      // Fallback: approximate voltage from raw value
      analog_data[0] = (float)raw_value * 3.3f / ((1 << ADC_BITWIDTH) - 1);
    }
  } else {
    // No calibration: approximate voltage from raw value
    analog_data[0] = (float)raw_value * 3.3f / ((1 << ADC_BITWIDTH) - 1);
  }

  ESP_LOGD(TAG, "ADC Raw: %d, Voltage: %.3fV", raw_value, analog_data[0]);

  return analog_data;
}

void ei_analog_sensor_deinit(void) {
  // Cleanup calibration
  if (adc_cali_handle != NULL) {
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(adc_cali_handle);
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
#endif
    adc_cali_handle = NULL;
  }

  // Cleanup ADC unit
  if (adc_handle != NULL) {
    adc_oneshot_del_unit(adc_handle);
    adc_handle = NULL;
  }

  ESP_LOGI(TAG, "Analog sensor deinitialized");
}
