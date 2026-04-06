# 🚁 Flight Test Documentation

[cite_start]This folder contains the logs and visual evidence of the drone's flight performance. [cite_start]Each test flight was designed to validate specific system behaviors, including stability, manual control responsiveness, and failsafe mechanisms.

## 📋 Test Objectives
Before every flight, a pre-flight checklist was followed to ensure system integrity:
* [cite_start]**IMU Calibration:** Validating sensor fusion accuracy using the MPU6050.
* [cite_start]**Link Quality:** Ensuring stable WebSocket communication between the ESP32-S3 and the controller.
* **Safety Failsafes:** Verifying motor cut-off logic in case of signal loss.

## 🎥 Flight Logs (Google Drive)
Due to the high resolution of the flight footage, all videos are hosted on Google Drive for optimal viewing.

**[Click here to view the Flight Test Videos](https://drive.google.com/drive/folders/1_4wGRGo9z4cDiCo5s-rIBpyP9PDAXBDx?usp=sharing)**

### Key Highlights:
1. [cite_start]**Indoor Hover Stability:** Testing the PID controller's ability to maintain a steady altitude.
2. **Manual Maneuvering:** Validating the responsiveness of the drone to commands sent via the mobile app.
3. [cite_start]**Telemetry Sync:** Real-time data visualization on the Raspberry Pi 5 dashboard while the drone is airborne[cite: 17].

## ⚙️ Post-Flight Analysis
[cite_start]After each flight, telemetry data such as orientation angles, battery voltage, and motor PWM signals were reviewed. This data-driven approach allowed for precise hardware adjustments and firmware optimizations to improve overall flight characteristics.
