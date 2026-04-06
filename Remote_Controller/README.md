
# 📱 Remote Control Interface

This interface provides full manual control over the drone via a web-based or mobile application. It communicates directly with the ESP32 flight controller to manage flight dynamics and system states.

## 🕹️ Control Features
* **Omnidirectional Joystick:** Maps user input to Roll and Pitch angles for precise maneuvering.
* **Vertical Throttle Slider:** Linear control of motor thrust (0 to 100%).
* **Real-time Configuration:** Dedicated buttons for **PID Tuning** and **Sensor Calibration** without needing to reflash the firmware.
* **Safety Controls:** Includes a dedicated "Start/Stop" and "Shut Down" mechanism for emergency situations.

## 📡 Communication Logic
The interface sends control packets (likely via WebSockets or UDP) containing:
1. **Axis Data:** Normalized X/Y coordinates from the joystick.
2. **Throttle Level:** 0-100% mapped to 1000-2000µs PWM signals.
3. **Command Flags:** Toggle states for Camera, PID mode, and Calibration sequences.
