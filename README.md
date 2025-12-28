# Drone-Project

## Project Overview
This project implements a real-time flight controller for a quadcopter based on the ESP32 microcontroller.
The system integrates control algorithms, sensor fusion, and wireless communication into a complete
hardware–firmware solution, developed and tested on a real platform.

## System Architecture
- ESP32-S3 microcontroller
- MPU6050 IMU (accelerometer and gyroscope)
- PID-based attitude control
- Wi-Fi communication for control and telemetry
- Motor drivers and ESCs

## Key Features
- Real-time PID flight control
- Sensor fusion using IMU data
- Wireless control and telemetry over Wi-Fi
- Full hardware–firmware integration

## Repository Structure
/esp32              – ESP32 flight controller firmware (C++)
/project_book       – Project documentation and report
/raspberry_pi_app   – Raspberry Pi 5 control application
/mit_app_inventor   – Mobile remote control application (MIT App Inventor source)
 

## Tools & Technologies
- Languages: C++, Python
- Protocols: I2C, UART, PWM, WebSockets
- Algorithms: Kalman Filter, PID Control, Sensor Fusion
- Platforms: ESP32-S3, Raspberry Pi 5, MIT App Inventor
