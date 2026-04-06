//ESP32 RC RECEIVER
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

//Servo pins
#define THROTTLE_SERVO_PIN 25
#define STEERING_SERVO_PIN 26
#define RX_BATTERY_PIN 34

//LED Control Pins
#define FRONT_LIGHT_PIN 13
#define LEFT_FRONT_TURN_PIN 27
#define RIGHT_FRONT_TURN_PIN 32
#define LEFT_BACK_LIGHT_PIN 33
#define RIGHT_BACK_LIGHT_PIN 21

//GPS Setup
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

//Servo settings
#define SERVO_MIN 500
#define SERVO_MAX 2500  
#define SERVO_CENTER 1500

//Servo angles
#define STEERING_LEFT_ANGLE  180
#define STEERING_RIGHT_ANGLE 0
#define STEERING_CENTER_ANGLE 90
#define THROTTLE_FORWARD_ANGLE  180
#define THROTTLE_REVERSE_ANGLE  0
#define THROTTLE_CENTER_ANGLE   90

//ESC PWM Detection
#define PWM_NEUTRAL 1500
#define PWM_THRESHOLD 50
#define PWM_BRAKE_THRESHOLD 1450

//Transmitter MAC address in Hex
uint8_t transmitterMAC[] = {0x70, 0x70, 0x07, 0xE6, 0xC3, 0x6C};

//Data structures
typedef struct 
{
  int throttle;
  int steering;
  uint32_t timestamp;
  uint8_t seq_num;
  float txBattery;
  uint8_t ledMode;  //0=Off, 1=20%, 2=100%
  uint8_t turnSignal; //0=Left, 1=Off, 2=Right
} ControlData;
typedef struct 
{
  float speed;
  float rxBattery;
  float rpm;
  uint8_t satellites;
  uint8_t seq_num;
} TelemetryData;
ControlData rxData;
TelemetryData txTelemetry;
Servo throttleServo, steeringServo;
unsigned long lastReceiveTime = 0;
unsigned long lastTelemetryTime = 0;
const long telemetryInterval = 100;
float totalDistance = 0.0;
unsigned long lastGPSUpdate = 0;

//LED CONTROL
int lightMode = 0; //0=Off, 1=20%, 2=100% Front/20% Back
int turnSignalMode = 1; //0=Left, 1=Off, 2=Right
bool brakeActive = false;
bool motorNeutral = true;
bool motorReverse = false;
bool manualBrakeActive = false;

//Turn signal timing
bool blinkState = false;
unsigned long blinkTimer = 0;
const int blinkInterval = 500;

//Brake timing
unsigned long brakeTimer = 0;
const unsigned long brakeDuration = 4000;

//PWM Variables
unsigned long pwmTimer = 0;
bool pwmState = false;
const int pwmOnTime = 4; // 4ms ON for 20% duty cycle (4/(4+16)=20%)
const int pwmOffTime = 16; // 16ms OFF

//Battery Reading Variables
float lastRxBatteryVoltage = 12.0; //Start with nominal voltage
unsigned long lastBatteryReadTime = 0;
const unsigned long batteryReadInterval = 2000; //Read battery every 2 seconds
const float BATTERY_MIN = 9.9;
const float BATTERY_MAX = 12.6;

//Calibration
const float CAL_M = (12.6 - 10.5) / (10.04 - 7.91);
const float CAL_B = 10.5 - CAL_M * 7.91;
const float DIVIDER_RATIO = 21.606;

//Map control percentage to servo angle
int controlToAngle(int controlPercent, int minAngle, int maxAngle, int centerAngle) 
{
  if (controlPercent >= 0) {
    return map(controlPercent, 0, 100, centerAngle, maxAngle);
  } else {
    return map(controlPercent, -100, 0, minAngle, centerAngle);
  }
}

//Battery Reading Functions
void updateBatteryReading() 
{
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBatteryReadTime >= batteryReadInterval) {
    lastBatteryReadTime = currentMillis;
    
    int adcValue = analogRead(RX_BATTERY_PIN);
    
    float espVoltage = adcValue * 3.3 / 4095.0;
    float rawVoltage = espVoltage * DIVIDER_RATIO;
    float newVoltage = CAL_M * rawVoltage + CAL_B;
    
    //low-pass filter for stability
    static float filteredVoltage = 12.0;
    filteredVoltage = filteredVoltage * 0.8 + newVoltage * 0.2;
    lastRxBatteryVoltage = filteredVoltage;
  }
}

float getRxBatteryVoltage() 
{
  return lastRxBatteryVoltage;
}

int calculateBatteryPercentage(float voltage) 
{
  if (voltage <= BATTERY_MIN) return 0;
  if (voltage >= BATTERY_MAX) return 100;
  return (int)((voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100.0);
}

int getBatteryPercentage() 
{
  return calculateBatteryPercentage(lastRxBatteryVoltage);
}

//RPM Functions
float calculateRPM(int throttlePercent) 
{
  float rpm = abs(throttlePercent) * 80.0;
  
  static float lastRPM = 0;
  if (throttlePercent == 0) 
  {
    rpm = 0;
  } 
  else if (rpm > lastRPM) 
  {
    rpm = lastRPM + (rpm - lastRPM) * 0.3;
  } 
  else 
  {
    rpm = lastRPM + (rpm - lastRPM) * 0.5;
  }
  
  lastRPM = rpm;
  return rpm;
}

//LED Functions
void setupLEDs() 
{
  Serial.println("Initializing LED Controller...");
  
  pinMode(FRONT_LIGHT_PIN, OUTPUT);
  pinMode(LEFT_FRONT_TURN_PIN, OUTPUT);
  pinMode(RIGHT_FRONT_TURN_PIN, OUTPUT);
  pinMode(LEFT_BACK_LIGHT_PIN, OUTPUT);
  pinMode(RIGHT_BACK_LIGHT_PIN, OUTPUT);
  
  //All OFF
  digitalWrite(FRONT_LIGHT_PIN, HIGH);
  digitalWrite(LEFT_FRONT_TURN_PIN, HIGH);
  digitalWrite(RIGHT_FRONT_TURN_PIN, HIGH);
  digitalWrite(LEFT_BACK_LIGHT_PIN, HIGH);
  digitalWrite(RIGHT_BACK_LIGHT_PIN, HIGH);
  
  Serial.println("LED Controller Ready");
}

void detectBrakeState(int throttleAngle) {
  int throttlePulse = map(throttleAngle, 0, 180, SERVO_MIN, SERVO_MAX);
  
  if (throttlePulse > (PWM_NEUTRAL - PWM_THRESHOLD) && throttlePulse < (PWM_NEUTRAL + PWM_THRESHOLD)) 
  {
    motorNeutral = true;
    motorReverse = false;
  } 
  else if (throttlePulse < PWM_NEUTRAL) 
  {
    motorNeutral = false;
    motorReverse = true;
  } 
  else 
  {
    motorNeutral = false;
    motorReverse = false;
  }
  
  static float lastRPM = 0;
  float currentRPM = calculateRPM(rxData.throttle);
  
  if (lastRPM > 2500.0 && currentRPM < 1000.0 && currentRPM > 0) 
  {
    manualBrakeActive = true;
    brakeTimer = millis();
    Serial.println("Brake lights ACTIVATED");
  }
  
  if (manualBrakeActive && (millis() - brakeTimer > brakeDuration)) 
  {
    manualBrakeActive = false;
    Serial.println("Brake lights DEACTIVATED");
  }
  
  lastRPM = currentRPM;
  brakeActive = manualBrakeActive || (motorReverse && throttlePulse < PWM_BRAKE_THRESHOLD);
}

void updateLEDs() 
{
  unsigned long currentMillis = millis();
  
  //Update turn signal blink
  if (currentMillis - blinkTimer >= blinkInterval) 
  {
    blinkState = !blinkState;
    blinkTimer = currentMillis;
  }
  
  //Update PWM timing
  if (currentMillis - pwmTimer >= (pwmState ? pwmOnTime : pwmOffTime)) 
  {
    pwmTimer = currentMillis;
    pwmState = !pwmState;
  }

  static bool frontLightState = HIGH;
  bool newFrontState;
  
  if (lightMode == 0) 
  {
    newFrontState = HIGH; //OFF
  } 
  else if (lightMode == 1) 
  {
    //20% ON
    if (pwmState) 
    {
      newFrontState = LOW; //ON
    } 
    else 
    {
      newFrontState = HIGH; //OFF
    }
  }
  else if (lightMode == 2) 
  {
    newFrontState = LOW; //100% ON
  }
  else 
  {
    newFrontState = HIGH; //OFF
  }
  
  //Only update if state changed
  if (newFrontState != frontLightState) 
  {
    digitalWrite(FRONT_LIGHT_PIN, newFrontState);
    frontLightState = newFrontState;
  }
  
  //Turn Signals
  static bool lastLeftTurnState = HIGH;
  static bool lastRightTurnState = HIGH;
  
  if (turnSignalMode == 0) //Left
  {
    bool newLeftState = blinkState ? LOW : HIGH;
    if (newLeftState != lastLeftTurnState) 
    {
      digitalWrite(LEFT_FRONT_TURN_PIN, newLeftState);
      lastLeftTurnState = newLeftState;
    }
    if (lastRightTurnState != HIGH) 
    {
      digitalWrite(RIGHT_FRONT_TURN_PIN, HIGH);
      lastRightTurnState = HIGH;
    }
  }
  else if (turnSignalMode == 2) //Right
  {
    bool newRightState = blinkState ? LOW : HIGH;
    if (newRightState != lastRightTurnState) 
    {
      digitalWrite(RIGHT_FRONT_TURN_PIN, newRightState);
      lastRightTurnState = newRightState;
    }
    if (lastLeftTurnState != HIGH) 
    {
      digitalWrite(LEFT_FRONT_TURN_PIN, HIGH);
      lastLeftTurnState = HIGH;
    }
  }
  else //OFF
  {
    if (lastLeftTurnState != HIGH) 
    {
      digitalWrite(LEFT_FRONT_TURN_PIN, HIGH);
      lastLeftTurnState = HIGH;
    }
    if (lastRightTurnState != HIGH) 
    {
      digitalWrite(RIGHT_FRONT_TURN_PIN, HIGH);
      lastRightTurnState = HIGH;
    }
  }
  
  //Back Lights
  static bool leftBackState = HIGH;
  static bool rightBackState = HIGH;
  
  if (brakeActive) 
  {
    //Brake active
    digitalWrite(LEFT_BACK_LIGHT_PIN, LOW);
    digitalWrite(RIGHT_BACK_LIGHT_PIN, LOW);
    leftBackState = LOW;
    rightBackState = LOW;
  }
  else 
  {
    //Left blight
    bool newLeftBackState;
    
    if (turnSignalMode == 0 && blinkState) 
    {
      //Left turn signal blinking
      newLeftBackState = LOW;
    } 
    else 
    {
      //Normal back light behavior (20% when lightMode is 1 or 2)
      if (lightMode == 1 || lightMode == 2) {
        //Both position 1 and 2: Back lights at 20%
        if (pwmState) 
        {
          newLeftBackState = LOW; //ON
        } 
        else 
        {
          newLeftBackState = HIGH; //OFF
        }
      }
      else 
      {
        //Position 0: Blights OFF
        newLeftBackState = HIGH;
      }
    }
    
    //Only update if state changed
    if (newLeftBackState != leftBackState) 
    {
      digitalWrite(LEFT_BACK_LIGHT_PIN, newLeftBackState);
      leftBackState = newLeftBackState;
    }
    
    //Right blight
    bool newRightBackState;
    
    if (turnSignalMode == 2 && blinkState) 
    {
      //Right turn signal blinking
      newRightBackState = LOW;
    } 
    else 
    {
      //Normal back light behavior (20% when lightMode is 1 or 2)
      if (lightMode == 1 || lightMode == 2) 
      {
        //Both position 1 and 2: Back lights at 20%
        if (pwmState) 
        {
          newRightBackState = LOW; //ON
        } 
        else 
        {
          newRightBackState = HIGH; //OFF
        }
      }
      else 
      {
        //Position 0: Back lights OFF
        newRightBackState = HIGH;
      }
    }
    
    //Only update if state changed
    if (newRightBackState != rightBackState) 
    {
      digitalWrite(RIGHT_BACK_LIGHT_PIN, newRightBackState);
      rightBackState = newRightBackState;
    }
  }
}

//ESP callback
void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) 
{
  lastReceiveTime = millis();
  
  if (len == sizeof(ControlData)) 
  {
    memcpy(&rxData, incomingData, sizeof(ControlData));
    
    //Update LED commands
    lightMode = rxData.ledMode;
    turnSignalMode = rxData.turnSignal;
    
    //Map to servo angles
    int throttleAngle = controlToAngle(rxData.throttle, THROTTLE_REVERSE_ANGLE, THROTTLE_FORWARD_ANGLE, THROTTLE_CENTER_ANGLE);
    int steeringAngle = controlToAngle(rxData.steering, STEERING_RIGHT_ANGLE, STEERING_LEFT_ANGLE, STEERING_CENTER_ANGLE);
    int throttlePulse = map(throttleAngle, 0, 180, SERVO_MIN, SERVO_MAX);
    int steeringPulse = map(steeringAngle, 0, 180, SERVO_MIN, SERVO_MAX);
    
    throttleServo.writeMicroseconds(throttlePulse);
    steeringServo.writeMicroseconds(steeringPulse);
    
    detectBrakeState(throttleAngle);
    
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) 
    {
      int batteryPercent = getBatteryPercentage();
      
      Serial.printf("Control: T=%3d%%, S=%3d%%, Bat=%.2fV(%d%%), LED=%d, Turn=%d, Brake=%d\n", rxData.throttle, rxData.steering, lastRxBatteryVoltage, batteryPercent, lightMode, turnSignalMode, brakeActive);
      lastDebug = millis();
    }
  }
}

void setupESPNowForTelemetry() 
{
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
}

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==================================================");
  Serial.println("RC RECEIVER");
  Serial.println("Light Modes:");
  Serial.println("  0 = All OFF");
  Serial.println("  1 = Front 20%, Back 20%");
  Serial.println("  2 = Front 100%, Back 20%");
  Serial.println("==================================================");
  
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  
  throttleServo.attach(THROTTLE_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  steeringServo.attach(STEERING_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  
  throttleServo.writeMicroseconds(SERVO_CENTER);
  steeringServo.writeMicroseconds(SERVO_CENTER);
  
  //Init Battery Monitoring
  pinMode(RX_BATTERY_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  setupLEDs();
  
  WiFi.mode(WIFI_STA);
  
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("ESP-NOW init failed!");
    ESP.restart();
  }
  
  esp_now_register_recv_cb(onDataReceived);
  setupESPNowForTelemetry();
  
  Serial.println("\nSystem Ready - Waiting for transmitter...");
}

void loop() 
{
  if (gpsSerial.available() > 0) 
  {
    gps.encode(gpsSerial.read());
  }

  updateLEDs();
  updateBatteryReading();
  
  //Telemetry
  static unsigned long lastFastLoop = 0;
  if (millis() - lastFastLoop > 10) 
  {
    lastFastLoop = millis();
    
    if (millis() - lastTelemetryTime >= telemetryInterval) 
    {
      lastTelemetryTime = millis();  
      txTelemetry.seq_num++;
      
      if (gps.speed.isValid()) 
      {
        txTelemetry.speed = gps.speed.kmph();
      } 
      else 
      {
        txTelemetry.speed = 0.0;
      }
      
      txTelemetry.rxBattery = lastRxBatteryVoltage;
      txTelemetry.rpm = calculateRPM(rxData.throttle);
      
      if (gps.satellites.isValid()) 
      {
        txTelemetry.satellites = gps.satellites.value();
      } 
      else 
      {
        txTelemetry.satellites = 0;
      }
      
      uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      esp_now_send(broadcastAddr, (uint8_t *)&txTelemetry, sizeof(txTelemetry));
    }
    
    //Failsafe
    if (millis() - lastReceiveTime > 1000 && lastReceiveTime > 0) {
      throttleServo.writeMicroseconds(SERVO_CENTER);
      steeringServo.writeMicroseconds(SERVO_CENTER);
    }
  }
  
  //delay (none rn)
}
