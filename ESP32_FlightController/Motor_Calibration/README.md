
# ⚙️ Motor Calibration & PWM Setup (ESP32-S3)

This module handles the initialization and calibration of the four Brushless DC motors via Electronic Speed Controllers (ESCs) using the ESP32 LED Control (LEDC) peripheral.

## 🔌 Hardware Configuration
* **Microcontroller:** ESP32-S3
* **PWM Channels:** 4 independent channels (0-3)
* **Output Pins:** GPIOs 5, 6, 9, 10
* **Resolution:** 12-bit (0-4095) for high-precision throttle control.
* **Frequency:** 250Hz (Optimized for standard multirotor ESCs).

## 📝 Calibration Logic & Calculations
The calibration follows the standard ESC handshake protocol. Since we are using a **12-bit resolution** at **250Hz**, the duty cycle calculation is as follows:
* **Period:** $1 / 250Hz = 4000\mu s$
* **Steps:** $2^{12} = 4096$ steps
* **Multiplier:** $4096 / 4000 \approx 1.024$
* **Formula:** `DutyCycle = 1.024 * PulseWidth_in_microseconds`

## 💻 Full Calibration Source Code
```cpp
/*
 * Drone Motor Calibration Logic
 * This code ramps the motors up and down to ensure synchronization.
 */

// ESC & Motor Definitions
const int escPins[4] = {5, 6, 9, 10};    // Motor Pins
const int pwmChannels[4] = {0, 1, 2, 3}; // PWM Channels
const int pwmFreq = 250;                 // 250Hz Frequency
const int pwmResolution = 12;            // 12-bit Resolution (0-4095)

// Microsecond range for ESC (Standard BLDC)
const int minPulseWidth = 1000; // IDLE / Stop
const int maxPulseWidth = 2000; // Full Throttle

void setup() {
  // Initialize each motor channel
  for (int i = 0; i < 4; i++) {
    ledcSetup(pwmChannels[i], pwmFreq, pwmResolution);
    ledcAttachPin(escPins[i], pwmChannels[i]);
    
    // Send minimum pulse (1000us) to arm the ESC safely
    ledcWrite(pwmChannels[i], 1.024 * minPulseWidth);
  }
  delay(2000); // Wait for ESC arming sequence
}

void loop() {
  // Gradual Ramp-Up from 1000us to 2000us
  for (int pulse = minPulseWidth; pulse <= maxPulseWidth; pulse += 50) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(pwmChannels[i], 1.024 * pulse);
    }
    delay(200);
  }

  // Gradual Ramp-Down from 2000us back to 1000us
  for (int pulse = maxPulseWidth; pulse >= minPulseWidth; pulse -= 50) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(pwmChannels[i], 1.024 * pulse);
    }
    delay(200);
  }
}
