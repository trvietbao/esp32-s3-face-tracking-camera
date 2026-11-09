#include "usb_stream.h"
#if __has_include("esp_driver_usb_serial_jtag.h")
#include "esp_driver_usb_serial_jtag.h"
#elif __has_include("driver/usb_serial_jtag.h")
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "usb_stream";

void usb_stream_init(void)
{
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = 16384,
        .rx_buffer_size = 512,
    };
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial JTAG driver install failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Native USB CDC Serial initialized");
    }
}

void usb_stream_send_frame(const uint8_t *jpeg_buf, size_t len, face_bbox_t bbox,
                           uint16_t t_pan_01us, uint16_t t_tilt_01us)
{
    if (!jpeg_buf || len == 0) return;

    // Extended Binary Protocol: Total 29 bytes header with AI & Servo hardware metrics
    // [0..1]   0xAA, 0xBB (Magic Header)
    // [2]      status (0x01: LOCKED, 0x02: TRACKING, 0x00: LOST)
    // [3..4]   x (Big-Endian uint16)
    // [5..6]   y (Big-Endian uint16)
    // [7..8]   w (Big-Endian uint16)
    // [9..10]  h (Big-Endian uint16)
    // [11..12] t_jpeg_01ms (Big-Endian uint16, unit 0.1ms)
    // [13..14] t_ai_01ms   (Big-Endian uint16, unit 0.1ms)
    // [15..16] t_pan_01us  (Big-Endian uint16, unit 0.1µs)
    // [17..18] t_tilt_01us (Big-Endian uint16, unit 0.1µs)
    // [19..22] JPEG Length (Big-Endian uint32)
    // [23..24] t_usb_01ms  (Big-Endian uint16, unit 0.1ms - measured write time)
    // [25..26] Free internal SRAM (Big-Endian uint16, unit KB)
    // [27..28] Free PSRAM (Big-Endian uint16, unit KB)
    uint8_t header[29];
    header[0] = 0xAA;
    header[1] = 0xBB;
    header[2] = bbox.status;

    header[3] = (uint8_t)((bbox.x >> 8) & 0xFF);
    header[4] = (uint8_t)(bbox.x & 0xFF);

    header[5] = (uint8_t)((bbox.y >> 8) & 0xFF);
    header[6] = (uint8_t)(bbox.y & 0xFF);

    header[7] = (uint8_t)((bbox.w >> 8) & 0xFF);
    header[8] = (uint8_t)(bbox.w & 0xFF);

    header[9] = (uint8_t)((bbox.h >> 8) & 0xFF);
    header[10] = (uint8_t)(bbox.h & 0xFF);

    header[11] = (uint8_t)((bbox.t_jpeg_01ms >> 8) & 0xFF);
    header[12] = (uint8_t)(bbox.t_jpeg_01ms & 0xFF);

    header[13] = (uint8_t)((bbox.t_ai_01ms >> 8) & 0xFF);
    header[14] = (uint8_t)(bbox.t_ai_01ms & 0xFF);

    header[15] = (uint8_t)((t_pan_01us >> 8) & 0xFF);
    header[16] = (uint8_t)(t_pan_01us & 0xFF);

    header[17] = (uint8_t)((t_tilt_01us >> 8) & 0xFF);
    header[18] = (uint8_t)(t_tilt_01us & 0xFF);

    header[19] = (uint8_t)((len >> 24) & 0xFF);
    header[20] = (uint8_t)((len >> 16) & 0xFF);
    header[21] = (uint8_t)((len >> 8) & 0xFF);
    header[22] = (uint8_t)(len & 0xFF);

    // Runtime memory telemetry (KB, measured with esp_heap_caps)
    uint16_t free_internal_kb = (uint16_t)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) >> 10);
    uint16_t free_psram_kb = (uint16_t)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10);
    header[25] = (uint8_t)((free_internal_kb >> 8) & 0xFF);
    header[26] = (uint8_t)(free_internal_kb & 0xFF);
    header[27] = (uint8_t)((free_psram_kb >> 8) & 0xFF);
    header[28] = (uint8_t)(free_psram_kb & 0xFF);

    // Measure real USB CDC write time (header + payload) with hardware timer.
    // t_usb of the *previous* frame is embedded in this header (real measured value).
    static uint16_t s_last_t_usb_01ms = 0;
    header[23] = (uint8_t)((s_last_t_usb_01ms >> 8) & 0xFF);
    header[24] = (uint8_t)(s_last_t_usb_01ms & 0xFF);

    int64_t t_usb_start = esp_timer_get_time();
    usb_serial_jtag_write_bytes(header, sizeof(header), pdMS_TO_TICKS(10));

    // Chunked write
    size_t written = 0;
    const size_t chunk_size = 512;
    while (written < len) {
        size_t to_write = (len - written > chunk_size) ? chunk_size : (len - written);
        int bytes = usb_serial_jtag_write_bytes(jpeg_buf + written, to_write, pdMS_TO_TICKS(15));
        if (bytes > 0) {
            written += bytes;
        } else {
            break;
        }
    }

    int64_t t_usb_us = esp_timer_get_time() - t_usb_start;
    s_last_t_usb_01ms = (uint16_t)((t_usb_us + 50) / 100); // 0.1ms unit
}
