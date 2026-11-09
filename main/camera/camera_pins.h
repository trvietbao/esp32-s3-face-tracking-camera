#pragma once

#include "esp_camera.h"

// Pinout mapping for ESP32-S3 WROOM Dual Type-C Board (Freenove / Standard S3-CAM)
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4   // SDA
#define CAM_PIN_SIOC    5   // SCL

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

// Target Resolution: 240x240 square frame
#define CAM_FRAME_SIZE   FRAMESIZE_240X240
#define CAM_PIXEL_FORMAT PIXFORMAT_JPEG

esp_err_t camera_init_240x240(void);
