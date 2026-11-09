#include "face_detect.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_jpeg_dec.h"
#include "human_face_detect.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>
#include <list>

static const char *TAG = "ai_face_espdl";

struct SonyRealTimeTracker {
    float x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f;
    uint32_t last_detect_ms = 0;
    int missed_frames = 0;
    uint8_t status = AF_STATUS_LOST;

    void update(bool detected, int det_x, int det_y, int det_w, int det_h, uint32_t now_ms) {
        if (detected) {
            x = (float)det_x; y = (float)det_y; w = (float)det_w; h = (float)det_h;
            status = AF_STATUS_LOCKED;
            missed_frames = 0;
            last_detect_ms = now_ms;
        } else {
            uint32_t elapsed = now_ms - last_detect_ms;
            // Hold prediction up to 300ms (~7 frames at 24FPS) to survive
            // brief misses from head pose/blur/squint instead of dropping lock.
            if (status != AF_STATUS_LOST && elapsed < 300 && missed_frames < 7) {
                status = AF_STATUS_TRACKING;
                missed_frames++;
            } else {
                status = AF_STATUS_LOST;
            }
        }

        w = (w < 16.0f) ? 16.0f : w;
        h = (h < 16.0f) ? 16.0f : h;
        x = (x < 0.0f) ? 0.0f : (x + w > 240.0f ? 240.0f - w : x);
        y = (y < 0.0f) ? 0.0f : (y + h > 240.0f ? 240.0f - h : y);
    }
};

static void crop_rgb888_roi(const uint8_t *src, int src_w,
                            int rx, int ry, int rw, int rh,
                            uint8_t *dst)
{
    int src_stride = src_w * 3;
    int dst_stride = rw * 3;
    for (int r = 0; r < rh; r++) {
        const uint8_t *s = src + (ry + r) * src_stride + (rx * 3);
        uint8_t *d = dst + r * dst_stride;
        memcpy(d, s, dst_stride);
    }
}

static SonyRealTimeTracker s_tracker;
static face_bbox_t s_latest_bbox = {
    .status = AF_STATUS_LOST, .has_face = false, .x = 0, .y = 0, .w = 0, .h = 0, .score = 0.0f,
    .t_jpeg_01ms = 0, .t_ai_01ms = 0
};
static portMUX_TYPE s_bbox_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t *s_infer_jpeg_buf = nullptr;
static size_t s_infer_jpeg_len = 0;
static SemaphoreHandle_t s_frame_ready_sem = nullptr;
static HumanFaceDetect *s_detector = nullptr;
static volatile bool s_ai_busy = false;

static void ai_inference_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing HumanFaceDetect on Core %d...", xPortGetCoreID());

    s_detector = new HumanFaceDetect(HumanFaceDetect::MSRMNP_S8_V1, false);
    s_detector->set_score_thr(0.20f, 0);
    s_detector->set_score_thr(0.20f, 1);
    ESP_LOGI(TAG, "HumanFaceDetect with hardware XTAL timer benchmarking loaded");

    const size_t rgb888_size = 240 * 240 * 3;
    uint8_t *rgb888_buf = (uint8_t *)heap_caps_aligned_alloc(16, rgb888_size, MALLOC_CAP_SPIRAM);
    if (!rgb888_buf) {
        ESP_LOGE(TAG, "Failed to allocate RGB888 buffer");
        vTaskDelete(NULL);
        return;
    }

    jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
    dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB888;
    jpeg_dec_handle_t jpeg_dec = nullptr;
    jpeg_dec_open(&dec_cfg, &jpeg_dec);

    const size_t max_roi_dim = 160;
    const size_t roi_buf_size = max_roi_dim * max_roi_dim * 3;
    uint8_t *roi_buf = (uint8_t *)heap_caps_aligned_alloc(16, roi_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!roi_buf) {
        roi_buf = (uint8_t *)heap_caps_malloc(roi_buf_size, MALLOC_CAP_SPIRAM);
    }

    uint16_t last_t_jpeg_01ms = 0;
    uint16_t last_t_ai_01ms = 0;
    int roi_tracking_frames = 0;

    while (1) {
        if (xSemaphoreTake(s_frame_ready_sem, portMAX_DELAY) == pdTRUE) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (s_infer_jpeg_len == 0 || !s_infer_jpeg_buf) {
                s_tracker.update(false, 0, 0, 0, 0, now_ms);
                s_ai_busy = false;
                continue;
            }

            // 1. Hardware XTAL timer: Measure SIMD 128-bit Assembly accelerated JPEG -> RGB888
            jpeg_dec_io_t io = {};
            io.inbuf = s_infer_jpeg_buf;
            io.inbuf_len = (int)s_infer_jpeg_len;
            io.outbuf = rgb888_buf;
            jpeg_dec_header_info_t header_info;

            int64_t t_dec_start = esp_timer_get_time();
            bool conv_ok = false;
            if (jpeg_dec && jpeg_dec_parse_header(jpeg_dec, &io, &header_info) == JPEG_ERR_OK) {
                conv_ok = (jpeg_dec_process(jpeg_dec, &io) == JPEG_ERR_OK);
            }
            int64_t t_dec_end = esp_timer_get_time();

            if (conv_ok) {
                last_t_jpeg_01ms = (uint16_t)((t_dec_end - t_dec_start + 50) / 100);
            } else {
                s_tracker.update(false, 0, 0, 0, 0, now_ms);
                s_ai_busy = false;
                continue;
            }

            bool use_roi = false;
            int rx = 0, ry = 0, rw = 240, rh = 240;

            if (s_tracker.status != AF_STATUS_LOST && s_tracker.w >= 20.0f && s_tracker.h >= 20.0f && roi_tracking_frames < 16) {
                float cx = s_tracker.x + s_tracker.w * 0.5f;
                float cy = s_tracker.y + s_tracker.h * 0.5f;
                // Generous padding (x1.8) so fast head motion stays inside the ROI
                float max_side = (s_tracker.w > s_tracker.h ? s_tracker.w : s_tracker.h) * 1.8f;
                int side = ((int)max_side + 7) & ~7;
                if (side < 96) side = 96;
                if (side > 160) side = 160;

                rx = (int)(cx - side * 0.5f);
                ry = (int)(cy - side * 0.5f);
                if (rx < 0) rx = 0;
                if (ry < 0) ry = 0;
                if (rx + side > 240) rx = 240 - side;
                if (ry + side > 240) ry = 240 - side;
                rw = side;
                rh = side;
                use_roi = true;
            }

            dl::image::img_t img;
            img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

            if (use_roi && roi_buf) {
                crop_rgb888_roi(rgb888_buf, 240, rx, ry, rw, rh, roi_buf);
                img.data = roi_buf;
                img.width = rw;
                img.height = rh;
            } else {
                img.data = rgb888_buf;
                img.width = 240;
                img.height = 240;
                rx = 0;
                ry = 0;
                rw = 240;
                rh = 240;
                roi_tracking_frames = 0;
            }

            // 2. Hardware XTAL timer: Measure ESP-DL Neural Network inference
            int64_t t_infer_start = esp_timer_get_time();
            auto &detect_results = s_detector->run(img);
            int64_t t_infer_end = esp_timer_get_time();
            last_t_ai_01ms = (uint16_t)((t_infer_end - t_infer_start + 50) / 100);

            float score = 0.0f;
            if (!detect_results.empty()) {
                auto &first = detect_results.front();
                int lx1 = (first.box[0] < 0) ? 0 : first.box[0];
                int ly1 = (first.box[1] < 0) ? 0 : first.box[1];
                int lx2 = (first.box[2] > rw) ? rw : first.box[2];
                int ly2 = (first.box[3] > rh) ? rh : first.box[3];

                int gx1 = (rx + lx1 < 0) ? 0 : rx + lx1;
                int gy1 = (ry + ly1 < 0) ? 0 : ry + ly1;
                int gx2 = (rx + lx2 > 240) ? 240 : rx + lx2;
                int gy2 = (ry + ly2 > 240) ? 240 : ry + ly2;

                int w = gx2 - gx1, h = gy2 - gy1;
                score = first.score;
                s_tracker.update((w > 0 && h > 0), gx1, gy1, w, h, now_ms);
                if (use_roi) roi_tracking_frames++;
            } else {
                // Detection miss in this frame: keep tracker in TRACKING state
                // (predictive hold) instead of forcing LOST immediately.
                if (use_roi) {
                    roi_tracking_frames++;
                } else {
                    roi_tracking_frames = 0; // retry ROI mode next frame
                }
                s_tracker.update(false, 0, 0, 0, 0, now_ms);
            }

            face_bbox_t new_bbox = {
                .status = s_tracker.status, .has_face = (s_tracker.status != AF_STATUS_LOST),
                .x = (uint16_t)s_tracker.x, .y = (uint16_t)s_tracker.y,
                .w = (uint16_t)s_tracker.w, .h = (uint16_t)s_tracker.h,
                .score = score, .t_jpeg_01ms = last_t_jpeg_01ms, .t_ai_01ms = last_t_ai_01ms
            };

            portENTER_CRITICAL(&s_bbox_mux);
            s_latest_bbox = new_bbox;
            portEXIT_CRITICAL(&s_bbox_mux);

            s_ai_busy = false;
        }
    }
}

extern "C" void ai_face_detect_init(void)
{
    s_frame_ready_sem = xSemaphoreCreateBinary();
    s_infer_jpeg_buf = (uint8_t *)heap_caps_aligned_alloc(16, 32 * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_infer_jpeg_buf) {
        s_infer_jpeg_buf = (uint8_t *)heap_caps_malloc(32 * 1024, MALLOC_CAP_SPIRAM);
    }
    xTaskCreatePinnedToCore(ai_inference_task, "ai_task", 16384, NULL, configMAX_PRIORITIES - 3, NULL, 1);
}

extern "C" void ai_face_detect_feed_frame(const uint8_t *jpeg_buf, size_t len)
{
    if (!jpeg_buf || len == 0 || !s_infer_jpeg_buf || s_ai_busy) return;
    if (len <= 32 * 1024) {
        s_ai_busy = true;
        memcpy(s_infer_jpeg_buf, jpeg_buf, len);
        s_infer_jpeg_len = len;
        xSemaphoreGive(s_frame_ready_sem);
    }
}

extern "C" face_bbox_t ai_face_detect_get_bbox(void)
{
    face_bbox_t res;
    portENTER_CRITICAL(&s_bbox_mux);
    res = s_latest_bbox;
    portEXIT_CRITICAL(&s_bbox_mux);
    return res;
}
