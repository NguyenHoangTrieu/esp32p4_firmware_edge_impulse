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

/* Include ----------------------------------------------------------------- */
#include "ei_microphone.h"
#include "ei_device_espressif_esp32.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF 6.0 I2S headers - NEW API
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "spi_flash_mmap.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

/* Private variables ------------------------------------------------------- */
static signed short *sampleBuffer;
static bool is_uploaded = false;
static int record_status = 0;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t audio_sampling_frequency = 16000;

static inference_t inference;

// NEW: I2S channel handle for ESP-IDF 6.0
static i2s_chan_handle_t rx_handle = NULL;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Audio thread setup */
#define AUDIO_THREAD_STACK_SIZE 4096
static const char* TAG = "AUDIO_PROVIDER";

/* Private functions ------------------------------------------------------- */

static void audio_write_callback(uint32_t n_bytes)
{
    EiDeviceESP32* dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    mem->write_sample_data((const uint8_t *)sampleBuffer, headerOffset + current_sample, n_bytes);
    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)sampleBuffer, n_bytes);

    current_sample += n_bytes;
    if(current_sample >= (samples_required << 1)) {
        i2s_deinit();
        record_status = false;
        free(sampleBuffer);
    }
}

static void audio_inference_callback(uint32_t n_bytes)
{
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) 
{
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    size_t bytes_read = 0;

    while (record_status) {
        // NEW API: i2s_channel_read instead of i2s_read
        esp_err_t ret = i2s_channel_read(rx_handle, sampleBuffer, 
                                         i2s_bytes_to_read, &bytes_read, 
                                         pdMS_TO_TICKS(100));

        if (ret != ESP_OK || bytes_read <= 0) {
            ESP_LOGE(TAG, "Error in I2S read: %s", esp_err_to_name(ret));
            continue;
        }

        if (bytes_read < i2s_bytes_to_read) {
            ESP_LOGW(TAG, "Partial I2S read: %d/%d bytes", bytes_read, i2s_bytes_to_read);
        }

        // Scale the data (amplify audio)
        for (int x = 0; x < bytes_read / 2; x++) {
            sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        }

        // Route to appropriate callback
        if (record_status == 1) {
            audio_write_callback(bytes_read);
        }
        else if (record_status == 2) {
            audio_inference_callback(bytes_read);
        }
        else {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void finish_and_upload(char *filename, uint32_t sample_length_ms) 
{
    EiDeviceESP32* dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    ei_printf("Done sampling, total bytes collected: %u\n", current_sample*2);
    dev->set_state(eiStateUploading);

    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 
              0, current_sample + headerOffset);
    ei_printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", 0);

    is_uploaded = true;
    dev->set_state(eiStateFinished);
    ei_printf("OK\n");
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }
    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    EiDeviceESP32* dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, 
                                         dev->get_sample_hmac_key().c_str());

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);
    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }

    // Find end of header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), 
                               end_of_header_ix);

    tr = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, 
                                          (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), 
                                          ref_size);
    if (tr != 0) {
        ei_printf("Failed to update signature from header (%d)\n", tr);
        return false;
    }

    end_of_header_ix += ref_size;

    int ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if (ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;
    return true;
}

/* Public functions -------------------------------------------------------- */

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{
    EiDeviceESP32* dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    start_delay_ms = start_delay_ms < 2000 ? 2000 : start_delay_ms;

    if (print_start_messages) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", start_delay_ms);
    }

    if(mem->erase_sample_data(0, (samples_required << 1) + 4096) != ((samples_required << 1) + 4096)) {
        return false;
    }

    vTaskDelay(start_delay_ms / portTICK_PERIOD_MS);

    create_header();
    record_status = 1;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, 
                (void*)AUDIO_THREAD_STACK_SIZE, 10, NULL);

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    return true;
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));
    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t));
    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    uint32_t sample_buffer_size = (n_samples / 100) * sizeof(int16_t);
    sampleBuffer = (int16_t *)ei_malloc(sample_buffer_size);
    if(sampleBuffer == NULL) {
        ei_free(inference.buffers[0]);
        ei_free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    audio_sampling_frequency = (uint32_t)(1000.f / interval_ms);

    if (i2s_init(audio_sampling_frequency)) {
        ei_printf("Failed to start I2S!");
    }

    ei_sleep(100);
    record_status = 2;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, 
                (void*)sample_buffer_size, 10, NULL);

    return true;
}

bool ei_microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf("Error sample buffer overrun. Decrease the number of slices per model window\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    inference.buf_ready = 0;
    return ret;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    return ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], 
                                     out_ptr, length);
}

bool ei_microphone_inference_end(void)
{
    record_status = 0;
    ei_sleep(100);
    i2s_deinit();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    ei_free(sampleBuffer);
    return true;
}

bool ei_microphone_sample_start(void)
{
    EiDeviceESP32* dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());
    EiDeviceMemory* mem = dev->get_memory();

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;
    is_uploaded = false;

    sampleBuffer = (int16_t *)ei_malloc(mem->block_size);
    if (sampleBuffer == NULL) {
        return false;
    }

    audio_sampling_frequency = (uint32_t)(1000.f / dev->get_sample_interval_ms());

    if (i2s_init(audio_sampling_frequency)) {
        ei_printf("Failed to start I2S!");
    }

    bool r = ei_microphone_record(dev->get_sample_length_ms(), 
                                   (((samples_required << 1)/ mem->block_size) * mem->block_erase_time), 
                                   true);
    if (!r) {
        return r;
    }

    while(record_status) {
        dev->set_state(eiStateSampling);
    }

    int ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, 
                                                    ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    finish_and_upload((char*)dev->get_sample_label().c_str(), dev->get_sample_length_ms());
    return true;
}

/* I2S Init/Deinit for ESP-IDF 6.0 ----------------------------------------- */

int i2s_init(uint32_t sampling_rate) 
{
    esp_err_t ret = ESP_OK;

    // 1. Create I2S channel (RX only for microphone)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear DMA buffer
    
    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return (int)ret;
    }

    // 2. Configure I2S standard mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampling_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_26,    // BCK
            .ws   = GPIO_NUM_32,    // LRCK/WS
            .dout = I2S_GPIO_UNUSED,
            .din  = GPIO_NUM_33,    // Data IN
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    // Adjust slot config for mono left channel
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        return (int)ret;
    }

    // 3. Enable the channel
    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        return (int)ret;
    }

    ESP_LOGI(TAG, "I2S initialized: %lu Hz, MONO", sampling_rate);
    return (int)ret;
}

int i2s_deinit(void) 
{
    if (rx_handle != NULL) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        ESP_LOGI(TAG, "I2S deinitialized");
    }
    return 0;
}