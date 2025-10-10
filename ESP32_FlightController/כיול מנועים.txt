// הגדרות מנועים (ESCים)
const int escPins[4] = {5, 6, 9, 10}; // פינים לכל מנוע
const int pwmChannels[4] = {0, 1, 2, 3}; // ערוצי PWM שונים
const int pwmFreq = 250;        // תדר 250Hz
const int pwmResolution = 12;   // רזולוציה 12 ביט (0-4095)

// טווח מיקרו־שניות ל-ESC
const int minPulseWidth = 1000; // עצירה
const int maxPulseWidth = 2000; // מקסימום

void setup() {
  // הגדרת כל מנוע
  for (int i = 0; i < 4; i++) {
    ledcSetup(pwmChannels[i], pwmFreq, pwmResolution);
    ledcAttachPin(escPins[i], pwmChannels[i]);
    // שליחת אות מינימום לאתחול ה־ESC
    ledcWrite(pwmChannels[i], 1.024 * minPulseWidth);
  }
  delay(2000); // זמן לאתחול ה־ESCים
}

void loop() {
  // עלייה הדרגתית
  for (int pulse = minPulseWidth; pulse <= maxPulseWidth; pulse += 50) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(pwmChannels[i], 1.024 * pulse);
    }
    delay(200);
  }

  // ירידה הדרגתית
  for (int pulse = maxPulseWidth; pulse >= minPulseWidth; pulse -= 50) {
    for (int i = 0; i < 4; i++) {
      ledcWrite(pwmChannels[i], 1.024 * pulse);
    }
    delay(200);
  }
}
