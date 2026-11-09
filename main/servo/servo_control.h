#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO pins
#define SERVO_PAN_PIN        1  // Pan (trục ngang)
#define SERVO_TILT_PIN       2  // Tilt (trục dọc)

// Giới hạn góc hoạt động an toàn (độ)
#define SERVO_PAN_MIN_DEG    10.0f
#define SERVO_PAN_MAX_DEG    170.0f
#define SERVO_TILT_MIN_DEG   25.0f
#define SERVO_TILT_MAX_DEG   155.0f

// Cấu hình chiều quay (1 = đảo chiều, 0 = giữ nguyên)
#define SERVO_PAN_INVERT     0
#define SERVO_TILT_INVERT    1

// Tốc độ quét tracking (độ/giây)
#define SERVO_PAN_SPEED_DPS  45.0f
#define SERVO_TILT_SPEED_DPS 45.0f

/**
 * @brief Khởi tạo phần cứng PWM LEDC 50Hz cho servo pan-tilt.
 */
esp_err_t servo_control_init(void);

/**
 * @brief Cập nhật vị trí tracking servo theo tọa độ khuôn mặt.
 * @param has_face Cờ báo phát hiện khuôn mặt hợp lệ
 * @param center_x Tọa độ tâm X của khuôn mặt (0 - 240)
 * @param center_y Tọa độ tâm Y của khuôn mặt (0 - 240)
 * @param out_pan_01us Thời gian thực thi trục Pan (đơn vị 0.1 µs)
 * @param out_tilt_01us Thời gian thực thi trục Tilt (đơn vị 0.1 µs)
 */
void servo_control_update(bool has_face, int center_x, int center_y,
                          uint16_t *out_pan_01us, uint16_t *out_tilt_01us);

/**
 * @brief Đặt góc trực tiếp cho 2 servo (đơn vị độ).
 */
void servo_control_set_angles(float pan_angle, float tilt_angle);

/**
 * @brief Đọc góc hiện tại của 2 servo.
 */
void servo_control_get_angles(float *out_pan, float *out_tilt);

#ifdef __cplusplus
}
#endif
