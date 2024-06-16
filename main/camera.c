/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include <esp_log.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "camera.h"
#include "globals.h"

#define FB_COUNT 2
#define HTTP_MAX_CLIENTS 10

#define PART_BOUNDARY "putin-khuylo"

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

typedef struct {
    httpd_req_t* req;
    size_t index;
} camera_http_client_t;

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

static const char *TAG = "DB_CAMERA";

static SemaphoreHandle_t frame_sync = NULL;
static char* frame_buffer = NULL;
static size_t frame_buffer_size = 0;
static size_t frame_buffer_length = 0;
static uint64_t frame_number = 0;

static TaskHandle_t camera_task_handle;
static TaskHandle_t http_client_task_handles[HTTP_MAX_CLIENTS] = { NULL };

static void camera_task(void* params) {
    ESP_LOGI(TAG, "Entering camera task");
    TickType_t last_wake_time = xTaskGetTickCount();;
    const TickType_t freq = pdMS_TO_TICKS(1000 / DB_CAM_FPS);

    frame_sync = xSemaphoreCreateMutex();

    for (;;) {
        ESP_LOGD(TAG, "Attempting to capture a frame");
        int64_t fr_start = esp_timer_get_time();

        camera_fb_t* fb = esp_camera_fb_get();
        size_t fb_l = fb->len;

        xSemaphoreTake(frame_sync, portMAX_DELAY);
        if (frame_buffer_size < fb_l) {
            if (frame_buffer != NULL) free(frame_buffer);
            /* frame_buffer = heap_caps_malloc(fb_l, MALLOC_CAP_SPIRAM); */
            ESP_LOGI(TAG, "Reallocating local frame buffer to %d bytes", fb_l);
            frame_buffer = malloc(fb_l);
            frame_buffer_size = fb_l;
        }
        frame_buffer_length = fb_l;
        memcpy(frame_buffer, fb->buf, fb_l);
        esp_camera_fb_return(fb);
        frame_number++;
        xSemaphoreGive(frame_sync);
        int64_t fr_end = esp_timer_get_time();

        ESP_LOGD(TAG, "Frame %llu captured: %lu KB, took %lld ms", frame_number, (uint32_t)(fb_l/1024), (fr_end - fr_start) / 1000);

        for (size_t client = 0; client < HTTP_MAX_CLIENTS; client++) {
            if (http_client_task_handles[client] != NULL)
                xTaskNotifyGive(http_client_task_handles[client]);
        }
        vTaskDelayUntil(&last_wake_time, freq);
    }
}

static void client_handler_task(void* params) {
    esp_err_t res = ESP_OK;
    char * part_buf[64]; // TODO: can be optimalized to strlen(_STREAM_PART) + n

    camera_http_client_t* client = (camera_http_client_t*) params;
    ESP_LOGI(TAG, "Starting a HTTP client handler task #%d", client->index);
    httpd_req_t* req = client->req;
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t fr_start = esp_timer_get_time();
        uint64_t fr = frame_number;

        ESP_LOGD(TAG, "HTTP client handler task #%d woken up, sending frame %llu", client->index, fr);

        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) break;

        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, frame_buffer_size);

        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res != ESP_OK) break;

        xSemaphoreTake(frame_sync, portMAX_DELAY);
        res = httpd_resp_send_chunk(req, (const char *)frame_buffer, frame_buffer_size);
        xSemaphoreGive(frame_sync);
        if (res != ESP_OK) break;

        int64_t fr_end = esp_timer_get_time();
        ESP_LOGD(TAG, "HTTP client handler task #%d done sending frame %llu, took %lld ms", client->index, frame_number, (fr_end - fr_start) / 1000);
    }

    ESP_LOGI(TAG, "Exiting a HTTP client handler task #%d", client->index);
    httpd_req_async_handler_complete(req);
    http_client_task_handles[client->index] = NULL;
    free(client);
    vTaskDelete(NULL);
}

esp_err_t camera_init() {
    camera_config_t camera_config = {
        .pin_pwdn = DB_CAM_PIN_PWDN,
        .pin_reset = DB_CAM_PIN_RESET,
        .pin_xclk = DB_CAM_PIN_XCLK,
        .pin_sccb_sda = DB_CAM_PIN_SIOD,
        .pin_sccb_scl = DB_CAM_PIN_SIOC,

        .pin_d7 = DB_CAM_PIN_Y9,
        .pin_d6 = DB_CAM_PIN_Y8,
        .pin_d5 = DB_CAM_PIN_Y7,
        .pin_d4 = DB_CAM_PIN_Y6,
        .pin_d3 = DB_CAM_PIN_Y5,
        .pin_d2 = DB_CAM_PIN_Y4,
        .pin_d1 = DB_CAM_PIN_Y3,
        .pin_d0 = DB_CAM_PIN_Y2,

        .pin_vsync = DB_CAM_PIN_VSYNC,
        .pin_href = DB_CAM_PIN_HREF,
        .pin_pclk = DB_CAM_PIN_PCLK,

        // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = DB_CAM_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        // We're interested in streaming a compressed (vs raw format)
        .pixel_format = PIXFORMAT_JPEG,

        .frame_size = DB_CAM_FRAME_SIZE,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

        .jpeg_quality = DB_CAM_JPG_QUALITY, //0-63, for OV series camera sensors, lower number means higher quality
        .fb_count = FB_COUNT, // Run in continuous mode
        /* .fb_location = CAMERA_FB_IN_PSRAM, */
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    if (DB_CAM_PIN_SIOC == DB_CAM_PIN_SIOD) {
        ESP_LOGW(TAG, "Camera init aborted, SIOC == SIOD - Configure first!");
        return ESP_FAIL;
    }
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) return err;

    xTaskCreate(camera_task, "db_camera", 4096, NULL, 1, &camera_task_handle);

    return ESP_OK;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

esp_err_t camera_frame_get_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %lu KB %lu ms", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    return res;
}


esp_err_t camera_stream_get_handler(httpd_req_t* req) {
    esp_err_t res = ESP_OK;
    size_t index;

    for (index = 0; index < HTTP_MAX_CLIENTS && http_client_task_handles[index] != NULL; index++);
    if (index == HTTP_MAX_CLIENTS - 1) {
        httpd_resp_set_status(req, "503 Busy");
        return ESP_OK;
    }

    camera_http_client_t* client = malloc(sizeof(camera_http_client_t));
    client->index = index;
    res = httpd_req_async_handler_begin(req, &client->req);
    if (res != ESP_OK) return res;

    ESP_LOGD(TAG, "Creating a HTTP client handler task #%d", index);
    xTaskCreate(client_handler_task, "db_camera_http_client", 4096,  client, 1, &http_client_task_handles[index]);

    return res;
}
