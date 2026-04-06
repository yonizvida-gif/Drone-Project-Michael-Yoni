# PID Tuning & Flight Stabilization Tests

This folder documents the iterative process of tuning the PID (Proportional, Integral, Derivative) constants to achieve stable and responsive flight for the ESP32-S3 quadcopter.

## 📌 Tuning Methodology
The tuning process involved isolating the drone's behavior by adjusting one parameter at a time. The goal was to eliminate high-frequency oscillations while ensuring the drone could maintain its attitude even under external disturbances.

## 📊 Flight Test Log
The following table summarizes the key configurations tested. Each link leads to a video demonstrating the drone's flight stability under those specific parameters.

| Test Phase | PID Constants (Kp, Ki, Kd) | Observations & Results | Video Link (Drive) |
| :--- | :--- | :--- | :--- |
| **Baseline Test** | Kp=0.5, Ki=0, Kd=0 | Low sensitivity. The drone reacts slowly to tilt and struggles to stay level. | [Watch Video](https://drive.google.com/drive/folders/1mEs3maFLdi-9PJG2mK-bnNHkeP8YyCYW?usp=sharing) |
| **Stability Iteration** | Kp=1.1, Ki=0.5, Kd=0 | Improved responsiveness, but suffers from "overshoot" and oscillations due to lack of damping. | [Watch Video]([YOUR_LINK_HERE](https://drive.google.com/drive/folders/1mEs3maFLdi-9PJG2mK-bnNHkeP8YyCYW?usp=sharing)) |
| **Final Calibration** | **Kp=1.1, Ki=0.45, Kd=0.003** | **Optimal balance. The D-term provides necessary damping, resulting in a stable and smooth hover.** | [Watch Video]([YOUR_LINK_HERE](https://drive.google.com/drive/folders/1mEs3maFLdi-9PJG2mK-bnNHkeP8YyCYW?usp=sharing)) |

## 🛠️ Parameter Functions
* **Kp (Proportional):** Controls the strength of the reaction to the current error. High Kp increases speed but causes oscillations.
* **Ki (Integral):** Eliminates steady-state errors (like wind or weight imbalance) by looking at the accumulated error over time.
* **Kd (Derivative):** Predicts future error by looking at the rate of change. It acts as a "brake" to prevent overshooting the target angle.
