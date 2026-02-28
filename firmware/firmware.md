# STM32 Autonomous Parking Firmware

## Overview

This repository contains the embedded firmware. The firmware provides real-time low-level control for vehicle actuation and sensor integration, and executes an autonomous parking maneuver using a deterministic state machine.

High-level commands and monitoring are handled externally with Raspberry Pi 5, while this firmware is responsible for motion control, sensor acquisition, safety handling, and telemetry feedback.

---

## System Architecture

<img width="576" height="249" alt="firmware_description" src="https://github.com/user-attachments/assets/9274a5f9-7a29-4e73-bbb4-d9b5481e7f47" />

The firmware is structured around the following functional layers:

- Hardware abstraction using STM32 HAL
- Timer-interrupt–driven real-time control loop
- Sensor acquisition (IMU, encoder, ultrasonic)
- Closed-loop motor speed control
- Steering control via PWM
- Autonomous parking state machine
- UART-based command and telemetry interface

---

## Control Logic

### Motor Speed Control

Motor speed is regulated using a PID controller with feed-forward compensation.

**Features:**
- Proportional, integral, and derivative terms
- Integral anti-windup clamping
- Feed-forward offset for ESC bias
- Output saturation to standard RC PWM range (1000–2000 µs)

The controller operates in a fixed-rate timer interrupt.

### Speed Ramping

To prevent abrupt changes in vehicle motion, target speed commands are applied through a linear ramping mechanism.  
Acceleration is limited by a fixed step size per control cycle.

### Steering Control

Steering is controlled using a PWM-driven servo.  
The desired steering angle is linearly mapped to PWM pulse width.  
Additional GPIO outputs are used for turn signal indication based on steering direction.

---

## Sensors and Feedback

### Inertial Measurement Unit (IMU)

- Interface: I²C
- Mode: Absolute orientation (NDOF)
- Data used: Yaw angle
- Resolution: 1/16 degree

Yaw data is used for orientation feedback during the parking maneuver, including relative rotation tracking with wrap-around handling.

### Wheel Encoder

- Implemented using a hardware timer in encoder mode
- Provides:
  - Instantaneous wheel speed
  - Traveled distance (odometry)

The encoder counter is reset at specific parking phases to simplify distance-based logic.

### Ultrasonic Distance Sensor

- Interface: UART
- Non-blocking trigger and reception
- Used for:
  - Parking space detection
  - Forward collision safety stop

---

## Autonomous Parking State Machine

The parking maneuver is implemented as a finite state machine:

| State | Description |
|------|-------------|
| PARK_IDLE | Waiting for external command |
| PARK_SEARCHING | Forward motion while scanning for a parking gap |
| PARK_POSITIONING | Forward motion to clear the detected gap |
| PARK_REVERSING | Reverse motion with fixed steering angle while monitoring yaw |
| PARK_FINISHED | Vehicle stopped, steering centered |

State transitions are driven by distance measurements, yaw angle changes, and encoder-based odometry.

---

## Communication Interfaces

### Command Input

- **Interface:** UART with DMA  
- **Command Format:** `S<speed>A<angle>`

**Parameters:**
- `speed` — Target vehicle speed in millimeters per second (mm/s)  
- `angle` — Steering angle in degrees
### Telemetry Output

The firmware periodically transmits telemetry data over UART, including:
- Actual vehicle speed
- Current yaw angle
- Measured distance to obstacle
- Current parking state

---

## Timing and Execution

- **Main loop**
- Sensor polling
- Parking state machine execution
- Telemetry transmission

- **Timer interrupt (TIM4)**
- Speed ramping
- Encoder processing
- PID computation
- Safety checks
- PWM updates

This separation ensures deterministic real-time behavior.

---

## Safety Mechanisms

- Automatic braking when an obstacle is detected below a minimum distance threshold
- Forced ESC neutral output when target speed is zero
- Integral reset when vehicle is stationary
- PWM output clamping to safe bounds

---

## Hardware Requirements

- STM32 microcontroller
- DC motor with ESC
- Steering servo
- Incremental wheel encoder
- IMU sensor (BNO055 or equivalent)
- Ultrasonic distance sensor (US-100)
- External controller (e.g., Raspberry Pi)

---

## Notes

- Designed for STM32CubeMX-generated projects
- Fully interrupt-safe and non-blocking
- Suitable for autonomous driving research, competitions, and prototyping

---
