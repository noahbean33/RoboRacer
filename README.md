# Robotics Control System

This repository contains embedded C code for a **robotics control system** using the **Arduino Portenta H7 (STM32H747XI)** microcontroller. The system handles **real-time motor control, line-following, and obstacle detection** using multiple sensors and communication protocols.

## Features
- **Low-Level API for Embedded Control**
- **Real-Time Sensor Data Processing**
- **PID-Based Line Following Algorithm**
- **PWM Motor Control**
- **UART-Based Communication Protocol**
- **LiDAR-Based Obstacle Detection**
- **Emergency Stop Logic**

---

## File Descriptions

### 1. `low_level_api.c`
This file implements the **low-level embedded API** for controlling motors, processing sensor data, and handling communication protocols. Key functionalities include:

- **Real-Time Sensor Updates** via timer interrupts for IR and LiDAR sensors.
- **UART Communication** with a **packet-based command protocol** and checksum validation.
- **PID-Based Line Following** using five IR sensors and a weighted moving average filter.
- **PWM Motor Control** for setting speed and adjusting heading.
- **Emergency Stop Logic** triggered by obstacle detection or communication loss.
- **Watchdog Timer Implementation** to ensure robust system operation.

### 2. `line_following_array.c`
This file manages **line following** using a **five-IR-sensor array**. The key components are:

- **Sensor Layout:** Five sensors arranged in a row to track the line.
- **Circular Buffer for Filtering:** Stores the last five readings for each sensor.
- **Majority-Vote Filtering:** Reduces noise using a software-based median filter.
- **Line Detection Logic:**
  - **Centered on Line** → Robot maintains position.
  - **Line on Left Side** → Robot steers **left**.
  - **Line on Right Side** → Robot steers **right**.
- **Serial Output for Debugging** displaying both raw and filtered sensor values.

### 3. `obstacle_detection_array.c`
This file implements **real-time obstacle detection** using a **TFmini Plus LiDAR** sensor. Features include:

- **Serial Communication with LiDAR** using `SoftwareSerial`.
- **9-Byte Data Frame Processing** to extract LiDAR distance measurements.
- **Circular Buffer for Filtering** with **median filtering** for stability.
- **Obstacle Detection Logic:**
  - If an obstacle is detected **within 800 cm (8 meters)**, an **emergency stop is triggered**.
  - Prints both **raw** and **filtered** LiDAR distances for debugging.

---

## Hardware Requirements
- **Arduino Portenta H7 (STM32H747XI)**
- **IR Sensor Array (x5) for Line Following**
- **TFmini Plus LiDAR for Obstacle Detection**
- **Motor Driver and PWM-Controlled Motors**
- **UART for Serial Communication**

---

## Installation & Setup
### 1. Clone the Repository
```sh
git clone https://github.com/your-username/robotics-control-system.git
cd robotics-control-system
```
### 2. Compile and Flash Code
Ensure you have the **STM32 HAL Library** and necessary dependencies installed. Use your **STM32 development environment** (e.g., STM32CubeIDE or Arduino IDE with STM32 support) to compile and upload the code.

### 3. Connect Peripherals
- **IR Sensors** to digital input pins.
- **TFmini Plus LiDAR** to serial communication pins.
- **Motor Driver** to PWM output pins.

---

## Usage
### 1. Line Following Mode
- The robot follows a **black line on a white background**.
- Adjusts steering dynamically based on **IR sensor feedback**.

### 2. Obstacle Detection Mode
- LiDAR continuously scans for obstacles **within 800 cm**.
- Triggers **emergency stop** if an obstacle is detected.

### 3. UART Commands (For Debugging)
Send the following UART commands to test functionality:
```c
CMD_SET_SPEED         = 0x01  // Set motor speed
CMD_ADJUST_HEADING    = 0x02  // Adjust steering angle
CMD_STOP              = 0x03  // Stop motors
CMD_GET_IR_DATA       = 0x04  // Get IR sensor data
CMD_GET_OBSTACLE_DATA = 0x05  // Get LiDAR obstacle data
```

---

## License
This project is licensed under the **MIT License**.

---

## Authors
Developed by **Your Name**. For questions or contributions, open an issue or pull request on GitHub.

