# SentraA1

<div align="center">
  <img src="https://img.shields.io/badge/Embedded-Systems-0A66C2?style=for-the-badge&logo=arm&logoColor=white" alt="Embedded Systems" />
  <img src="https://img.shields.io/badge/Electronics-Design-FF6B35?style=for-the-badge&logo=electronics&logoColor=white" alt="Electronics" />
  <img src="https://img.shields.io/badge/EdgeAI-Ready-2E8B57?style=for-the-badge&logo=ai&logoColor=white" alt="Edge AI" />
  <img src="https://img.shields.io/badge/IoT-Prototype-8A2BE2?style=for-the-badge&logo=iot&logoColor=white" alt="IoT Prototype" />
  <img src="https://img.shields.io/badge/Safety-Alert-System-DC143C?style=for-the-badge&logo=shield&logoColor=white" alt="Safety Alert System" />
</div>

A polished embedded systems project built around an STM32 Blue Pill microcontroller for vehicle speed monitoring and theft-trigger alerting. This firmware combines interrupt-driven speed sensing, touch-based activation logic, relay control, and UART event reporting in a compact and extensible PlatformIO project.

The system is designed to detect motion across a defined sensing distance, calculate the speed of a passing object or vehicle, and raise an alarm when a tampering or theft event is triggered. It is well-suited for prototyping, demonstrations, and further expansion into smarter vehicle or access-control applications.

This project also forms the embedded edge of a larger intelligent monitoring pipeline that pairs the STM32 firmware with a Raspberry Pi 4B, a 16MP camera, and AI-based vehicle classification for real-world event logging and review.

---

## Overview

SentraA1 is a firmware project that integrates two primary functions:

1. Speed detection using a dual-sensor timing mechanism
2. Theft/trigger detection using a touch input that activates a relay and alarm output

The project is implemented in C++ for the STM32 platform and is managed through PlatformIO for easy building, flashing, and debugging.

---

## Key Features

- Interrupt-based speed measurement with a configurable sensing distance
- Real-time overspeed event handling
- Touch-triggered security logic with delay and active-period control
- Relay and alarm activation for physical response
- UART output in structured JSON format for downstream systems
- Clean modular source structure with dedicated driver files

---

## Full System Architecture

SentraA1 is designed as a complete edge intelligence solution that extends beyond the STM32 firmware:

- The STM32 microcontroller detects an overspeed event and transmits the data over UART to a Raspberry Pi 4B.
- The Raspberry Pi wakes its camera subsystem, captures a high-resolution snapshot using a 16MP camera, and prepares the evidence for processing.
- A YOLOv8-powered inference pipeline performs AI-based vehicle type classification.
- The resulting event data, including speed, vehicle classification, timestamp, and image reference, is sent to Supabase for storage and later review.

### Raspberry Pi Companion Side

The Raspberry Pi companion code is located in [RaspberryPi_Python](RaspberryPi_Python). It includes scripts and utilities for:

- camera capture and image acquisition
- UART event handling from the STM32 device
- Supabase logging and data upload
- testing and experimentation for the complete monitoring pipeline

---

## Hardware Concept

The firmware is designed around the following core hardware behavior:

- Two sensing inputs detect the passage of an object across a measured distance
- A touch input initiates the protection sequence
- A relay output provides a switching action
- An alarm output signals the active state
- UART transmits event data to a connected host or controller

### Pin Mapping

| Function | Pin |
| --- | --- |
| Sensor A | PA0 |
| Sensor B | PA1 |
| Touch Input | PB6 |
| Relay Output | PB0 |
| Alarm Output | PA6 |

---

## Software Architecture

The project is organized into clear functional modules:

- [src/main.cpp](src/main.cpp) — main application loop and system initialization
- [src/speedDriver.cpp](src/speedDriver.cpp) / [src/speedDriver.h](src/speedDriver.h) — speed sensing and timing logic
- [src/theftTrigger.cpp](src/theftTrigger.cpp) / [src/theftTrigger.h](src/theftTrigger.h) — touch-triggered theft logic
- [src/uartDriver.cpp](src/uartDriver.cpp) / [src/uartDriver.h](src/uartDriver.h) — UART communication and event formatting

---

## Project Structure

```text
SentraA1/
├── include/                # Project assets and media
├── src/                    # Firmware source files
│   ├── main.cpp
│   ├── speedDriver.cpp
│   ├── speedDriver.h
│   ├── theftTrigger.cpp
│   ├── theftTrigger.h
│   ├── uartDriver.cpp
│   └── uartDriver.h
├── test/                   # Test folder placeholder
├── platformio.ini          # PlatformIO configuration
└── README.md               # Project documentation
```

---

## Getting Started

### Prerequisites

- VS Code
- PlatformIO IDE extension
- An STM32 Blue Pill board
- USB serial connection for flashing

### Build and Upload

1. Open the project folder in VS Code
2. Ensure PlatformIO is installed and configured
3. Build the firmware:

```bash
pio run
```

4. Upload to the board:

```bash
pio run --target upload
```

---

## Usage Notes

- The speed detector uses a fixed sensing distance of 0.5 meters in the current implementation.
- Speed values are calculated in kilometers per hour and can be adjusted based on the physical installation.
- The theft trigger includes a debounce period and a configurable activation window.
- UART output is emitted as a JSON object for easy integration with a host device or monitoring system.

Example UART payload:

```json
{"event":"vehicle_detected","speed_kmh":12.5,"touch":true,"source":"stm32-001"}
```

---

## Media Gallery

The project images and demo video are stored in [include/project_images](include/project_images).

### Physical Device Images

![Device view 1](include/project_images/WhatsApp%20Image%202026-06-13%20at%2009.24.30.jpeg)

![Device view 2](include/project_images/WhatsApp%20Image%202026-06-13%20at%2009.24.31.jpeg)

![Device view 3](include/project_images/WhatsApp%20Image%202026-06-13%20at%2009.24.32%20(1).jpeg)

![Device view 4](include/project_images/WhatsApp%20Image%202026-06-13%20at%2009.24.32.jpeg)



## Future Improvements

Potential enhancements for this project include:

- Adding a more advanced speed calibration system
- Supporting wireless communication such as Bluetooth or Wi-Fi
- Creating a PCB-based version for a cleaner final product
- Integrating a mobile or web dashboard for live monitoring
- Adding EEPROM-based configuration for thresholds and timings

---

## License

This project is intended for educational, prototyping, and personal development use. Please adapt and extend it responsibly based on your own hardware setup and requirements.

---

## Acknowledgment

This repository reflects a practical embedded systems implementation that combines sensing, control, and communication in a single compact firmware solution. It is a strong foundation for further innovation and product development.
