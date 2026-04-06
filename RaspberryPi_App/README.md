
# 🖥️ Raspberry Pi Ground Station Dashboard ("Drone Cockpit")

This folder contains the **Ground Control Station (GCS)** application, developed in Python for the Raspberry Pi 5. It manages real-time telemetry visualization and system monitoring for the ESP32-S3 quadcopter.

## 📊 Dashboard Preview
The application features a custom-built HUD (Heads-Up Display) that integrates live sensor data with a dynamic 3D-like drone model.

![Drone Cockpit Dashboard](dashboard.png)

## 🚀 Technical Highlights
* **Asynchronous Networking:** Built with `websockets` and `asyncio` to handle high-speed, full-duplex data streaming with minimal latency.
* **Multi-Threaded Architecture:** Utilizes `QThread` to decouple the WebSocket data processing from the PyQt5 UI thread, ensuring a smooth 60fps experience even during heavy data bursts.
* **Vector Graphics Engine:** The drone's orientation is rendered in real-time using `QPainter`. It calculates Pitch/Roll visualization using trigonometric functions based on IMU telemetry.
* **Smart Data Parsing:** Capable of handling multiple telemetry formats (JSON/CSV) and converting raw PWM signals (1000µs - 2000µs) into intuitive UI elements like battery percentages and degree-based angles.

## 🛠️ Features
* **Live 3D Orientation:** Synchronized visual feedback of the drone's actual flight attitude.
* **System Health:** Real-time monitoring of processor temperature, battery voltage levels, and throttle output.
* **Connection Management:** Dynamic connection status (Red/Yellow/Green) with built-in reconnection logic.

## 📦 Installation & Setup
To run the dashboard, ensure you have Python 3 installed, then set up the environment:

1. **Install Required Libraries:**
   ```bash
   pip install PyQt5 websockets
