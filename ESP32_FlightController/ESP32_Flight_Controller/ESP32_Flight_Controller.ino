#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ===== FreeRTOS (טסקים + נעילה) =====
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define SDA_PIN 20 // DATA for MPU
#define SCL_PIN 19 // CLK for MPU
#define BAT_PIN 14  // pin for BATTERY monitor ********************************************************************************************

#define led_Y_pin  12 // OFF LED
#define led_B_pin  13 // ON LED

const int freq = 250;         // תדר
const int resolution = 12;    // רזולוציה (0-4095)
const int channels[4] = {0, 1, 2, 3};  // ערוצי PWM
const int ESC_Pins[4] = {5, 6, 9, 10};  // pins PWM  // 5 - MOTOR1 , 6 - MOTOR2 , 9 - MOTOR3 , 10 - MOTOR4

// פרטי הרשת של הטלפון (Hotspot)
const char* ssid = "M"; //Fiber 4K-9680
const char* password = "12345678";
const float R1 = 51000.0; //******************************************************************************
const float R2 = 9500; //******************************************************************************

WebSocketsServer webSocket = WebSocketsServer(80); // מאזין בפורט 80

// ===== משתנים משותפים (מוגנים במנעול) =====
volatile int SHUTDOWN = 0;
int current_batt , last_batt=0; // ***************************************************************************************
int ReceiverValue[2] = {0,0}; // CONTROLLER
volatile int ReceiverValueSOCKET[2] = {0,0};

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

volatile float MPUTempC = 0;
volatile uint32_t lastBroadcastUs = 0;  // חותמת זמן לשידור האחרון (ל-rate limit)

volatile float InputThrottle; 
//uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch ;
float InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=1.1; float PRatePitch=PRateRoll; float PRateYaw=2; //0.6    1.7     1.72      2               2.5
float IRateRoll=0.65; float IRatePitch=IRateRoll; float IRateYaw=12; //3.5    3.2     2.5     2.5              0.75
float DRateRoll=0.003; float DRatePitch=DRateRoll; float DRateYaw=0; //0.03  0.012   0.015   0.025            0.12
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2.0; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

// ===== מנעול לשיתוף מצב =====
SemaphoreHandle_t ctrlMutex;

// ============================ פונקציות מקוריות שלך ============================
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}


int monitor_BATT(){ // function for monitoring the battery - return battery in percents  -------- need to check: pin number, value of ersistors R1 R2 ------------
 int percent;
 int read_bat = analogRead(BAT_PIN);
 float voltage = (((float)read_bat/4095)*3.3)*((R1+R2)/R2); //casting for analog voltage and cancel the voltage divider count
 if(voltage>=11.6)
 {
   percent = 100;
 } 
  else if(voltage>=11.3)
  {
    percent = 75;
  }
   else if(voltage>=10.85)
  {
    percent = 50;
  }
   else if(voltage>=10.55)
  {
    percent = 25;
  }

 else{
   percent = 0; 
 }
return percent;
}

void gyro_signals(void) {
  // Accel
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  // Acc
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // *** קריאת טמפרטורה  ***
  int16_t TempRaw = (Wire.read() << 8) | Wire.read();
  MPUTempC = (float)TempRaw / 340 + 36.53;

  // Gyro
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  // חישובים...
  RateRoll  = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw   = (float)GyroZ/65.5;

  AccX=(float)AccXLSB/4096 - 0.04;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096 - 0.05;

  AngleRoll  = atan(AccY/sqrtf(AccX*AccX+AccZ*AccZ))*(180/3.142);
  AnglePitch = -atan(AccX/sqrtf(AccY*AccY+AccZ*AccZ))*(180/3.142);
}


void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

// ========================= WebSocket Event =========================// ========================= WebSocket Event =========================
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){ // WEB SOCKET EVENT FUNCTION
  switch (type) 
  {
    case WStype_DISCONNECTED:
      digitalWrite(led_Y_pin, HIGH);
      digitalWrite(led_B_pin, LOW);
      break;
    case WStype_CONNECTED:
      digitalWrite(led_Y_pin, LOW);
      digitalWrite(led_B_pin, HIGH);
      break;   
  }

  if (type == WStype_TEXT) 
  {
    String msg = String((char *)payload);
           

    if (msg == "ON") // ARMING
     {
      if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
        SHUTDOWN = 0;
        InputThrottle=1000;
        ReceiverValueSOCKET[0] = 1500;
        ReceiverValueSOCKET[1] = 1500;
        xSemaphoreGive(ctrlMutex);
      }
     } 
    else if (msg == "OFF") // MOTORSֹ-OFF
     {
      if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
        InputThrottle=1000;
        xSemaphoreGive(ctrlMutex);
      }
     }
    else if (msg == "SHUTDOWN") // MOTORSֹ-OFF
     {
      if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
        SHUTDOWN = 1;
        xSemaphoreGive(ctrlMutex);
      }
     }

    // אם ההודעה כוללת פסיק => זו רשימת ערכים מהג’ויסטיק
    if (msg.indexOf(",") > 0)
    {
      int commaIndex = msg.indexOf(",");
      String xStr = msg.substring(1, commaIndex);
      String yStr = msg.substring(commaIndex + 1);

      int x = xStr.toInt() * 2.22222 + 1000 ; // CASTING FROM 0-450 TO 1000-2000
      int y = yStr.toInt() * -2.22222 + 2000; // CASTING FROM 0-450 TO 1000-2000 AND SWITCH BETWEEN THE AXIS
      if(x == 1499) x = 1500;


      if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
        ReceiverValueSOCKET[0] = x;
        ReceiverValueSOCKET[1] = y;
        xSemaphoreGive(ctrlMutex);
      }
    }

    else if(msg.indexOf(',') == -1 && msg.startsWith("[") && msg.endsWith("]")){
      int commaIndex = msg.indexOf("]");
      String tStr = msg.substring(1, commaIndex);
      int m = tStr.toInt() * -2.22222 + 2000; // CASTING FROM 0-450 TO 1000-2000 AND SWITCH BETWEEN THE AXIS
      if(m > 2000)  m = 2000;
      else if(m < 1000) m = 1000;
      
      if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
        InputThrottle = m;
        xSemaphoreGive(ctrlMutex);
      }
    }
  }


// === שידור ל-RPI בתדירות מקסימלית של פעם ב-10ms (100Hz) ===
uint32_t nowUs = micros();
if ((uint32_t)(nowUs - lastBroadcastUs) >= 7000) {  // בטוח גם עם overflow
  int thr = (int)InputThrottle;
  int rx0 = ReceiverValueSOCKET[0];
  int rx1 = ReceiverValueSOCKET[1];
  float tC = MPUTempC;  // מחושב ב-gyro_signals()
  int bat = monitor_BATT();
  String msgRPI5 = String(thr) + "," + String(rx0) + "," + String(rx1) + "," + String(tC, 2) + "," + String(bat);
  webSocket.broadcastTXT(msgRPI5);
  //Serial.print("battary: ");Serial.println(bat);
  lastBroadcastUs = nowUs;
}

    
  
}


// ========================= Tasks =========================

// WebSocket task (רץ על Core 0 עם WiFi/AsyncTCP)
void wsTask(void* pv) {
  for(;;) {
    webSocket.loop();
    monitor_BATT();
    vTaskDelay(pdMS_TO_TICKS(1)); // רספונסיביות טובה בלי לחנוק CPU
  }
}

// Control task (גוף ה-loop שלך) – 4ms

void controlTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(4); // 4ms => 250Hz
  TickType_t lastWake = xTaskGetTickCount();
  
  for(;;) {
    gyro_signals();

    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

    // --- קריאה אטומית של הקלטים המשותפים ---
    float localThrottle;
    int   localRX0, localRX1;
    int   localShutdown;
    if (xSemaphoreTake(ctrlMutex, portMAX_DELAY)) {
      localThrottle  = InputThrottle;
      localRX0       = ReceiverValueSOCKET[0];
      localRX1       = ReceiverValueSOCKET[1];
      localShutdown  = SHUTDOWN;
      xSemaphoreGive(ctrlMutex);
    }

    ReceiverValue[0] = localRX0;
    ReceiverValue[1] = localRX1;

    DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch= 0.10 * (ReceiverValue[1] - 1500);

    ErrorAngleRoll = DesiredAngleRoll  - KalmanAngleRoll;
    ErrorAnglePitch= DesiredAnglePitch - KalmanAnglePitch;

    // דליפת I-term כששקט וקרוב לאופק
    if (fabs(RateRoll)<1.0 && fabs(RatePitch)<1.0) {
      if (fabs(ErrorAngleRoll)  < 1.0) { PrevItermAngleRoll  *= 0.90f; PrevItermRateRoll  *= 0.90f; }
      if (fabs(ErrorAnglePitch) < 1.0) { PrevItermAnglePitch *= 0.90f; PrevItermRatePitch *= 0.90f; }
    }

    // איפוס מהיר כשהכול ממש קרוב ל-0 (מאיץ חזרה לערך היציב)
    if (fabs(RateRoll)<0.6 && fabs(RatePitch)<0.6 && fabs(ErrorAngleRoll)<0.6 && fabs(ErrorAnglePitch)<0.6) {
      PrevItermAngleRoll = PrevItermAnglePitch = 0;
      PrevItermRateRoll  = PrevItermRatePitch  = 0;
    }


    pid_equation(ErrorAngleRoll,  PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,  PrevItermAngleRoll);
    DesiredRateRoll    = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch     = PIDReturn[0];
    PrevErrorAnglePitch  = PIDReturn[1];
    PrevItermAnglePitch  = PIDReturn[2];

    ErrorRateRoll  = DesiredRateRoll  - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw  = 0  - RateYaw;

    if(ErrorRateRoll < 0.5 && ErrorRateRoll > -0.5) ErrorRateRoll = 0;
    if(ErrorRatePitch < 0.5 && ErrorRatePitch > -0.5) ErrorRatePitch = 0;

    pid_equation(ErrorRateRoll,  PRateRoll,  IRateRoll,  DRateRoll,  PrevErrorRateRoll,  PrevItermRateRoll);
    InputRoll         = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch          = PIDReturn[0];
    PrevErrorRatePitch  = PIDReturn[1];
    PrevItermRatePitch  = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw          = PIDReturn[0];
    PrevErrorRateYaw  = PIDReturn[1];
    PrevItermRateYaw  = PIDReturn[2];


    if (localThrottle > 1800) localThrottle = 1800;

    MotorInput1= 1.024*(localThrottle-InputRoll-InputPitch-InputYaw);
    MotorInput2= 1.024*(localThrottle-InputRoll+InputPitch+InputYaw);
    MotorInput3= 1.024*(localThrottle+InputRoll+InputPitch-InputYaw);
    MotorInput4= 1.024*(localThrottle+InputRoll-InputPitch+InputYaw);

    if (MotorInput1 > 2000) MotorInput1 = 1999;
    if (MotorInput2 > 2000) MotorInput2 = 1999;
    if (MotorInput3 > 2000) MotorInput3 = 1999;
    if (MotorInput4 > 2000) MotorInput4 = 1999;

    int ThrottleIdle=1180;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    int ThrottleCutOff=1000;
    if (localThrottle<1050 || localShutdown || digitalRead(led_Y_pin) == HIGH) {
      MotorInput1=ThrottleCutOff; 
      MotorInput2=ThrottleCutOff;
      MotorInput3=ThrottleCutOff; 
      MotorInput4=ThrottleCutOff;
      reset_pid();
    }

    ledcWrite(0,MotorInput1);
    ledcWrite(1,MotorInput2);
    ledcWrite(2,MotorInput3); 
    ledcWrite(3,MotorInput4);

    vTaskDelayUntil(&lastWake, period); // 4ms קבוע
  }
}


// ========================= setup / loop =========================
void setup() {
  Serial.begin(115200);

  ledcSetup(0, freq, resolution);
  ledcAttachPin(5, 0);
  ledcSetup(1, freq, resolution);
  ledcAttachPin(6, 1);
  ledcSetup(2, freq, resolution);
  ledcAttachPin(9, 2);
  ledcSetup(3, freq, resolution);
  ledcAttachPin(10, 3);


  pinMode(led_Y_pin, OUTPUT);
  pinMode(led_B_pin, OUTPUT);
  digitalWrite(led_Y_pin, HIGH);
  digitalWrite(led_B_pin, LOW); 

  Wire.setClock(400000);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000;RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
 
  pinMode(2, OUTPUT);  // LED flag for Calibration is done!
  digitalWrite(2, HIGH);
  delay(2000);
  digitalWrite(2, LOW);

  WiFi.begin(ssid, password);
  //Serial.print("מתחבר לרשת WiFi");                //serial print test for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  //Serial.println("\nWiFi מחובר");                // בדיקת עבור התחברות וויפיי - הדפסה סיריאלית  
  //Serial.print("כתובת IP: ");
  //Serial.println(WiFi.localIP());                // IP OF THE ESP BOARD

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // יצירת mutex
  ctrlMutex = xSemaphoreCreateMutex();
  if (!ctrlMutex) {
    Serial.println("Failed to create mutex!");
  }

  // === יצירת טסקים: WS על Core 0, Control על Core 1 ===
  xTaskCreatePinnedToCore(wsTask, "wsTask", 4096, NULL, 2, NULL, 0);          // WiFi/WS
  xTaskCreatePinnedToCore(controlTask, "controlTask", 8192, NULL, 3, NULL, 1); // Flight control
}

void loop() {
  // הכל רץ בטסקים
  vTaskDelay(pdMS_TO_TICKS(1000));
}