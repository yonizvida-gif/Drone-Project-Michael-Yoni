# 🚁 Flight Testing & System Validation

This folder documents the experimental validation of the quadcopter, moving from a controlled test bench environment to successful free-flight maneuvers.

## 📋 System Testing Methodology
To ensure safety and data integrity, the project followed a structured testing phase:

1. **Integrated Test Bench (Fixed Rig):**
   * **Setup:** The drone was secured to a custom-built safety rig, connected to the Raspberry Pi 5 Ground Station and the Mobile Controller.
   * **Validation:** Verified the end-to-end communication loop. You can see the ESP32-S3 receiving throttle and direction commands from the app while simultaneously streaming telemetry to the Raspberry Pi.
   * **Real-time Telemetry:** The dashboard displays live data including **Battery Temperature, Pitch/Roll Angles, Throttle Level, and Connection Status (Heartbeat).**

2. **Open Field Flight Tests:**
   * **Objective:** Validating the PID controller’s stability in an uncontrolled environment.
   * **Stability:** Demonstrated the drone's ability to maintain a level attitude and respond accurately to manual inputs without flipping or losing control.

## 🎥 Flight Logs & Demos (Google Drive)
All high-resolution test footage is hosted on Google Drive.
[**Click here to view the Flight Test Videos**](https://drive.google.com/drive/folders/1_4wGRGo9z4cDiCo5s-rIBpyP9PDAXBDx?usp=sharing)

### Video Guide:
* **Video 1 (System Integration):** Comprehensive demo of the full-stack system on the test rig (Mobile App -> ESP32 -> RPi 5).
* **Videos 2 & 3 (Free Flight):** Successful outdoor flights demonstrating attitude stability and maneuvering responsiveness.

## ⚙️ Post-Flight Analysis
The telemetry data viewed on the Raspberry Pi during these tests was used to fine-tune the PID constants, ensuring that the manual thrust response is linear and the orientation is stable under various conditions.
