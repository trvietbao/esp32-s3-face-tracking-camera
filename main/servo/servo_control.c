#include "servo_control.h"
#include "driver/ledc.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "servo_ctrl";

// Cấu hình LEDC PWM (50Hz, 14-bit)
#define LEDC_TIMER          LEDC_TIMER_1
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY_HZ   50
#define LEDC_PAN_CH         LEDC_CHANNEL_1
#define LEDC_TILT_CH        LEDC_CHANNEL_2

// Thông số xung servo chuẩn (500us - 2500us)
#define SERVO_MIN_US        500.0f
#define SERVO_MAX_US        2500.0f
#define PWM_PERIOD_US       20000.0f
#define PWM_MAX_DUTY        16384.0f

// Thông số vùng bám mục tiêu (khung hình 240x240)
#define CENTER_X            120.0f
#define CENTER_Y            120.0f
#define DEADBAND_PIXELS     20.0f
#define SLOWDOWN_ZONE       75.0f

// Bước cập nhật tối thiểu và thời gian giữ vị trí (3s)
#define MG996R_MIN_STEP_DEG 0.45f
#define LOST_HOLD_US        3000000

static float s_virtual_pan = 90.0f;
static float s_virtual_tilt = 90.0f;
static float s_hardware_pan = 90.0f;
static float s_hardware_tilt = 90.0f;
static int64_t s_last_update_us = 0;
static int64_t s_last_face_us = 0;
static bool s_initialized = false;

static inline float clamp_f(float val, float min_v, float max_v) {
  if (val < min_v) return min_v;
  if (val > max_v) return max_v;
  return val;
}

static uint32_t angle_to_duty(float angle_deg) {
  float clamped = clamp_f(angle_deg, 0.0f, 180.0f);
  float pulse_us = SERVO_MIN_US + (clamped / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)((pulse_us / PWM_PERIOD_US) * PWM_MAX_DUTY + 0.5f);
}

static inline uint32_t pan_angle_to_duty(float angle_deg) {
#if SERVO_PAN_INVERT
  angle_deg = 180.0f - angle_deg;
#endif
  return angle_to_duty(angle_deg);
}

static inline uint32_t tilt_angle_to_duty(float angle_deg) {
#if SERVO_TILT_INVERT
  angle_deg = 180.0f - angle_deg;
#endif
  return angle_to_duty(angle_deg);
}

static void apply_hardware_pwm(float pan_deg, float tilt_deg) {
  ledc_set_duty(LEDC_MODE, LEDC_PAN_CH, pan_angle_to_duty(pan_deg));
  ledc_update_duty(LEDC_MODE, LEDC_PAN_CH);
  ledc_set_duty(LEDC_MODE, LEDC_TILT_CH, tilt_angle_to_duty(tilt_deg));
  ledc_update_duty(LEDC_MODE, LEDC_TILT_CH);
}

static void update_axis_position(float err, float base_speed, float dt,
                                 float dir_sign, float min_deg, float max_deg,
                                 float *virtual_pos) {
  float dist = fabsf(err);
  if (dist <= DEADBAND_PIXELS) {
    return;
  }
  float speed = base_speed;
  if (dist < SLOWDOWN_ZONE) {
    float factor = (dist - DEADBAND_PIXELS) / (SLOWDOWN_ZONE - DEADBAND_PIXELS);
    speed = base_speed * clamp_f(factor, 0.35f, 1.0f);
  }
  float dir = (err > 0.0f) ? dir_sign : -dir_sign;
  *virtual_pos += dir * speed * dt;
  *virtual_pos = clamp_f(*virtual_pos, min_deg, max_deg);
}

static void update_return_to_center(float speed_dps, float dt,
                                    float *virtual_pan, float *virtual_tilt) {
  float step = speed_dps * dt * 0.6f;
  if (*virtual_pan > 90.0f + step) *virtual_pan -= step;
  else if (*virtual_pan < 90.0f - step) *virtual_pan += step;
  else *virtual_pan = 90.0f;

  if (*virtual_tilt > 90.0f + step) *virtual_tilt -= step;
  else if (*virtual_tilt < 90.0f - step) *virtual_tilt += step;
  else *virtual_tilt = 90.0f;
}

esp_err_t servo_control_init(void) {
  ESP_LOGI(TAG, "Initializing Pan-Tilt Servos (Pan: GPIO %d, Tilt: GPIO %d)",
           SERVO_PAN_PIN, SERVO_TILT_PIN);

  ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_DUTY_RES,
      .timer_num = LEDC_TIMER,
      .freq_hz = LEDC_FREQUENCY_HZ,
      .clk_cfg = LEDC_AUTO_CLK
  };
  esp_err_t err = ledc_timer_config(&timer_cfg);
  if (err != ESP_OK) return err;

  ledc_channel_config_t pan_cfg = {
      .gpio_num = SERVO_PAN_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_PAN_CH,
      .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER,
      .duty = pan_angle_to_duty(90.0f), .hpoint = 0
  };
  err = ledc_channel_config(&pan_cfg);
  if (err != ESP_OK) return err;

  ledc_channel_config_t tilt_cfg = {
      .gpio_num = SERVO_TILT_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_TILT_CH,
      .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER,
      .duty = tilt_angle_to_duty(90.0f), .hpoint = 0
  };
  err = ledc_channel_config(&tilt_cfg);
  if (err != ESP_OK) return err;

  s_virtual_pan = 90.0f;
  s_virtual_tilt = 90.0f;
  s_hardware_pan = 90.0f;
  s_hardware_tilt = 90.0f;
  s_last_update_us = esp_timer_get_time();
  s_last_face_us = s_last_update_us;
  s_initialized = true;
  return ESP_OK;
}

void servo_control_update(bool has_face, int center_x, int center_y,
                          uint16_t *out_pan_01us, uint16_t *out_tilt_01us) {
  if (!s_initialized) {
    if (out_pan_01us) *out_pan_01us = 0;
    if (out_tilt_01us) *out_tilt_01us = 0;
    return;
  }

  int64_t now_us = esp_timer_get_time();
  float dt = (s_last_update_us > 0) ? (float)(now_us - s_last_update_us) / 1000000.0f : 0.04f;
  s_last_update_us = now_us;
  if (dt > 0.1f) dt = 0.1f;

  if (has_face) {
    s_last_face_us = now_us;
    update_axis_position((float)center_x - CENTER_X, SERVO_PAN_SPEED_DPS, dt,
                         -1.0f, SERVO_PAN_MIN_DEG, SERVO_PAN_MAX_DEG, &s_virtual_pan);
    update_axis_position((float)center_y - CENTER_Y, SERVO_TILT_SPEED_DPS, dt,
                         1.0f, SERVO_TILT_MIN_DEG, SERVO_TILT_MAX_DEG, &s_virtual_tilt);
  } else if (now_us - s_last_face_us > LOST_HOLD_US) {
    update_return_to_center(SERVO_PAN_SPEED_DPS, dt, &s_virtual_pan, &s_virtual_tilt);
  }

  uint32_t c0 = esp_cpu_get_cycle_count();
  if (fabsf(s_virtual_pan - s_hardware_pan) >= MG996R_MIN_STEP_DEG) {
    s_hardware_pan = s_virtual_pan;
    ledc_set_duty(LEDC_MODE, LEDC_PAN_CH, pan_angle_to_duty(s_hardware_pan));
    ledc_update_duty(LEDC_MODE, LEDC_PAN_CH);
  }
  uint32_t c1 = esp_cpu_get_cycle_count();

  uint32_t c2 = esp_cpu_get_cycle_count();
  if (fabsf(s_virtual_tilt - s_hardware_tilt) >= MG996R_MIN_STEP_DEG) {
    s_hardware_tilt = s_virtual_tilt;
    ledc_set_duty(LEDC_MODE, LEDC_TILT_CH, tilt_angle_to_duty(s_hardware_tilt));
    ledc_update_duty(LEDC_MODE, LEDC_TILT_CH);
  }
  uint32_t c3 = esp_cpu_get_cycle_count();

  if (out_pan_01us) *out_pan_01us = (uint16_t)((c1 - c0) / 24);
  if (out_tilt_01us) *out_tilt_01us = (uint16_t)((c3 - c2) / 24);
}

void servo_control_set_angles(float pan_angle, float tilt_angle) {
  if (!s_initialized) return;
  s_virtual_pan = clamp_f(pan_angle, SERVO_PAN_MIN_DEG, SERVO_PAN_MAX_DEG);
  s_virtual_tilt = clamp_f(tilt_angle, SERVO_TILT_MIN_DEG, SERVO_TILT_MAX_DEG);
  s_hardware_pan = s_virtual_pan;
  s_hardware_tilt = s_virtual_tilt;
  apply_hardware_pwm(s_hardware_pan, s_hardware_tilt);
}

void servo_control_get_angles(float *out_pan, float *out_tilt) {
  if (out_pan) *out_pan = s_hardware_pan;
  if (out_tilt) *out_tilt = s_hardware_tilt;
}
