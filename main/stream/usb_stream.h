#pragma once

#include <stdint.h>
#include <stddef.h>
#include "face_detect.h"

void usb_stream_init(void);
void usb_stream_send_frame(const uint8_t *jpeg_buf, size_t len, face_bbox_t bbox,
                           uint16_t t_pan_01us, uint16_t t_tilt_01us);
