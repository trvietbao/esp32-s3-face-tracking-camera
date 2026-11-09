#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AF_STATUS_LOST = 0,     // No target / out of frame
    AF_STATUS_LOCKED = 1,   // Real-time Face Lock (High Confidence)
    AF_STATUS_TRACKING = 2  // Predictive Motion Tracking (Head tilt / blur)
} af_status_t;

typedef struct {
    uint8_t status;         // af_status_t (0, 1, 2)
    bool has_face;          // status != 0 (backward compatibility)
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    float score;
    uint16_t t_jpeg_01ms;   // Measured via hardware XTAL timer (0.1ms resolution)
    uint16_t t_ai_01ms;     // Measured via hardware XTAL timer (0.1ms resolution)
} face_bbox_t;

void ai_face_detect_init(void);
void ai_face_detect_feed_frame(const uint8_t *jpeg_buf, size_t len);
face_bbox_t ai_face_detect_get_bbox(void);

#ifdef __cplusplus
}
#endif
