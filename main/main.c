#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "camera_pins.h"
#include "usb_stream.h"
#include "face_detect.h"
#include "servo_control.h"

static const char *TAG = "main_app";

#define TARGET_FPS        24
#define FRAME_INTERVAL_MS (1000 / TARGET_FPS) // 41ms per frame

static void camera_stream_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Camera stream task started on Core %d at %d FPS", xPortGetCoreID(), TARGET_FPS);

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(FRAME_INTERVAL_MS);

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            // 1. Feed frame asynchronously to Core 1 for Face Detection
            ai_face_detect_feed_frame(fb->buf, fb->len);

            // 2. Retrieve latest available bounding box
            face_bbox_t bbox = ai_face_detect_get_bbox();

            // 3. Update Pan-Tilt Servo Tracking (GPIO 1: Pan, GPIO 2: Tilt) & measure latency
            uint16_t t_pan_01us = 0;
            uint16_t t_tilt_01us = 0;
            if (bbox.has_face && bbox.status != AF_STATUS_LOST && bbox.w > 0 && bbox.h > 0) {
                int cx = bbox.x + (bbox.w / 2);
                int cy = bbox.y + (bbox.h / 2);
                servo_control_update(true, cx, cy, &t_pan_01us, &t_tilt_01us);
            } else {
                servo_control_update(false, 0, 0, &t_pan_01us, &t_tilt_01us);
            }

            // 4. Send Frame + BBox + Servo Latencies via USB CDC OTG
            usb_stream_send_frame(fb->buf, fb->len, bbox, t_pan_01us, t_tilt_01us);
            esp_camera_fb_return(fb);
        }

        // Lock precise 24 FPS timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-S3 OV3660 OTG Streamer with Pan-Tilt Tracking...");
    ESP_LOGI(TAG, "Hardware Clock: CPU %d MHz | Flash %s | Octal PSRAM 80MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, CONFIG_ESPTOOLPY_FLASHSIZE);

    // Cáº£ 2 servo láº­p tá»©c quay vá» gÃ³c 90 Ä‘á»™ khi vá»«a khá»Ÿi Ä‘á»™ng
    servo_control_init();

    usb_stream_init();
    ai_face_detect_init();

    esp_err_t err = camera_init_240x240();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed, aborting task creation.");
        return;
    }

    // Camera Stream Task pinned to Core 0 with 4KB stack
    xTaskCreatePinnedToCore(
        camera_stream_task,
        "cam_stream",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        NULL,
        0
    );
}

