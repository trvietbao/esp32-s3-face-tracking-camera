# ESP32-S3 AI Face Tracking & Pan-Tilt Camera

An embedded AI camera system powered by the **ESP32-S3**, featuring on-device real-time face detection, 2-axis closed-loop Pan-Tilt servo tracking, UVC USB streaming, and an integrated web interface.

---

## Features

- **On-Device Neural Network Inference**:
  - Leverages Espressif's **ESP-DL** and `human_face_detect` models for real-time face detection directly on the microcontroller.
  - Multi-core FreeRTOS architecture: Video capture and streaming on Core 0, AI inference running asynchronously on Core 1 for minimal latency.
- **Closed-Loop Pan-Tilt Tracking**:
  - Automatically calculates bounding box deviations from image center.
  - Controls two PWM servos (Pan on GPIO 1, Tilt on GPIO 2) with smooth proportional tracking and bounds protection.
- **High-Framerate Video Streaming**:
  - Supports 24 FPS target video pipeline.
  - USB Video Class (UVC) stream implementation for plug-and-play webcam compatibility with PC/Linux.
- **Web Interface**:
  - Clean client UI (`web/`) to monitor live camera feed, view telemetry, and configure tracking parameters.

---

## Hardware Specification

- **MCU**: ESP32-S3 (with Octal/Quad PSRAM recommended)
- **Camera Sensor**: OV2640 / OV3660 DVP interface (configured in `main/camera/camera_pins.h`)
- **Actuators**:
  - Pan Servo (Horizontal): `GPIO 1`
  - Tilt Servo (Vertical): `GPIO 2`

---

## Project Structure

```text
├── main/
│   ├── ai/
│   │   ├── face_detect.cpp   # Neural network inference pipeline (ESP-DL)
│   │   └── face_detect.h
│   ├── camera/
│   │   ├── camera_init.c     # Camera driver initialization and DMA config
│   │   └── camera_pins.h     # Pin assignment for camera bus
│   ├── servo/
│   │   ├── servo_control.c   # 2-axis Pan-Tilt servo PWM driver and tracking logic
│   │   └── servo_control.h
│   ├── stream/
│   │   ├── usb_stream.c      # UVC USB video streaming backend
│   │   └── usb_stream.h
│   ├── CMakeLists.txt        # Component build rules
│   ├── idf_component.yml     # ESP Component Registry dependencies
│   └── main.c                # FreeRTOS tasks and orchestrator
├── web/                      # Embedded web streaming UI
│   ├── app.js
│   ├── index.html
│   └── style.css
├── CMakeLists.txt            # Top-level project CMake
├── partitions.csv            # Flash partition layout
├── sdkconfig.defaults        # Default ESP-IDF configuration (PSRAM, UVC, Camera)
└── README.md
```

---

## Getting Started

### Prerequisites

- [Espressif ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) (v5.0 or later)

### Build & Flash

1. Configure target for ESP32-S3:
   ```bash
   idf.py set-target esp32s3
   ```

2. Build the project:
   ```bash
   idf.py build
   ```

3. Flash and monitor:
   ```bash
   idf.py -p COMx flash monitor
   ```

---

## License

This project is licensed under the MIT License.
