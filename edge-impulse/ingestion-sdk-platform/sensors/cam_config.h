/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "model-parameters/model_metadata.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CAM_RGB565_BITS_PER_PIXEL           16
#define CAM_RGB888_BITS_PER_PIXEL           24
#define CAM_MIPI_IDI_CLOCK_RATE             (50000000)
#define CAM_MIPI_CSI_LANE_BITRATE_MBPS      200 //line_rate = pclk * 4

#ifdef __cplusplus
}
#endif
