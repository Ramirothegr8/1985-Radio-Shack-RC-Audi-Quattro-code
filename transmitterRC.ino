//ESP32 RC TRANSMITTER
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>

//LCD Display (SH1106 OLED)
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Control pins
#define THROTTLE_POT_PIN 34
#define STEERING_POT_PIN 35
#define TX_BATTERY_PIN 33

//Digital LED Toggle Switch Pins
#define LIGHT_POS1_PIN 17 //Light switch position 1 (Off)
#define LIGHT_POS3_PIN 4 //Light switch position 3 (100%)
#define TURN_POS1_PIN 16 //Turn signal position 1 (Left)
#define TURN_POS3_PIN 19 //Turn signal position 3 (Right)

//Calibration values
#define THROTTLE_RAW_FORWARD 976
#define THROTTLE_RAW_REVERSE 2297
#define THROTTLE_RAW_CENTER 1637
#define STEERING_RAW_RIGHT 2223
#define STEERING_RAW_LEFT 927
#define STEERING_RAW_CENTER 1584
#define THROTTLE_DEADBAND 30
#define STEERING_DEADBAND 30

//Receiver MAC address
uint8_t receiverMAC[] = {0x00, 0x70, 0x07, 0xE6, 0x8E, 0x9C};

//Data structures
typedef struct 
{
  int throttle; //-100 to +100
  int steering; //-100 to +100
  uint32_t timestamp;
  uint8_t seq_num;
  float txBattery; //Transmitter battery voltage
  uint8_t ledMode; //0=Off, 1=70%, 2=100%
  uint8_t turnSignal; //0=Left, 1=Off, 2=Right
} ControlData;
typedef struct 
{
  float speed; //GPS speed in km/h
  float rxBattery; //Receiver battery voltage
  float rpm; //Calculated RPM
  uint8_t satellites; //GPS satellites count
  uint8_t seq_num; //Sequence number for tracking
} TelemetryData;
ControlData txData;
TelemetryData rxTelemetry;
uint8_t sequence = 0;
unsigned long lastDisplayUpdate = 0;
const long displayInterval = 200;
unsigned long startTime = 0;
float tripDistance = 0.0;

//Display variables
int displayedRPM = 0;
int displayedSpeed = 0;
float displayedDistance = 0.0;
int rxBatteryPercent = 0;
int txBatteryPercent = 0;

//telemetry
bool telemetryReceived = false;
unsigned long lastTelemetryTime = 0;
const long telemetryTimeout = 3000;

//LED toggle states
int lightMode = 0;
int turnSignalMode = 1;

//RX Battery (3S LiPo) 
const float RX_BATTERY_MIN = 9.0; //3.0V per cell (empty)
const float RX_BATTERY_MAX = 12.245; //100% at 12.245V

//TX Battery (6x AAA Alkaline)
const float TX_CORRECTION_FACTOR = 1.566;

//TX Battery voltage range
const float TX_BATTERY_MIN = 6.0; //1.0V per cell (empty)
const float TX_BATTERY_MAX = 9.6; //1.6V per cell (full)

const int RX_BATTERY_SAMPLES = 20;
const int TX_BATTERY_SAMPLES = 30;

//RX battery averaging
float rxVoltageBuffer[RX_BATTERY_SAMPLES];
int rxBufferIndex = 0;
float rxVoltageSum = 0;

//TX battery averaging
float txVoltageBuffer[TX_BATTERY_SAMPLES];
int txBufferIndex = 0;
float txVoltageSum = 0;
float lastStableTxPercent = 0;
unsigned long lastTxUpdate = 0;
const long txUpdateInterval = 500; //Only update TX percentage every 500ms

/*
unsigned long lastSerialDebug = 0;
const long serialDebugInterval = 2000;
*/
//Draw prototypes
void drawDashboard(int rpm, int speed, float distance, int RxBatteryPercent, int TxBatteryPercent, int hours, int minutes, bool rxConnected);
void drawRPMGauge(int rpm);
void drawSpeedBox(int speed);
void drawDistanceBox(float distance);
void drawRxBar(int RxBatteryPercent, bool connected);
void drawTxBar(int TxBatteryPercent);
void drawTime(int hours, int minutes);
void drawAudiQuattroLogo();

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) 
{
}
//Callback when telemetry data is received from car
void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) 
{
  if (len == sizeof(TelemetryData)) 
  {
    memcpy(&rxTelemetry, incomingData, sizeof(TelemetryData));
    
    telemetryReceived = true;
    lastTelemetryTime = millis();
    
    displayedSpeed = (int)rxTelemetry.speed;
    displayedRPM = (int)rxTelemetry.rpm;

    float rxVoltage = rxTelemetry.rxBattery;
    
    rxVoltageSum -= rxVoltageBuffer[rxBufferIndex];
    rxVoltageBuffer[rxBufferIndex] = rxVoltage;
    rxVoltageSum += rxVoltage;
    rxBufferIndex = (rxBufferIndex + 1) % RX_BATTERY_SAMPLES;
    float avgRxVoltage = rxVoltageSum / RX_BATTERY_SAMPLES;
    
    if (avgRxVoltage <= RX_BATTERY_MIN) 
    {
      rxBatteryPercent = 0;
    } 
    else if (avgRxVoltage >= RX_BATTERY_MAX) 
    {
      rxBatteryPercent = 100;
    } 
    else 
    {
      rxBatteryPercent = (int)((avgRxVoltage - RX_BATTERY_MIN) / (RX_BATTERY_MAX - RX_BATTERY_MIN) * 100.0);
    }
    
    rxBatteryPercent = constrain(rxBatteryPercent, 0, 100);
    
    static unsigned long lastDistanceUpdate = 0;
    if (lastDistanceUpdate > 0) 
    {
      float timeHours = (millis() - lastDistanceUpdate) / 3600000.0;
      displayedDistance += rxTelemetry.speed * timeHours;
    }
    lastDistanceUpdate = millis();
  }
}

float readTxBatteryVoltage() 
{
  long totalADC = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) 
  {
    totalADC += analogRead(TX_BATTERY_PIN);
    delay(2);
  }
  int adcValue = totalADC / samples;
  
  float espVoltage = adcValue * 3.3 / 4095.0;
  float rawVoltage = espVoltage * 21.606;
  float correctedVoltage = rawVoltage * TX_CORRECTION_FACTOR;
  
  return correctedVoltage;
}

int calculateTxBatteryPercent() 
{
  static float filteredVoltage = 0;
  static bool firstRun = true;
  
  float currentVoltage = readTxBatteryVoltage();
  
  if (firstRun) 
  {
    filteredVoltage = currentVoltage;
    firstRun = false;
  } 
  else 
  {
    filteredVoltage = filteredVoltage * 0.9 + currentVoltage * 0.1;
  }
  
  txVoltageSum -= txVoltageBuffer[txBufferIndex];
  txVoltageBuffer[txBufferIndex] = filteredVoltage;
  txVoltageSum += filteredVoltage;
  txBufferIndex = (txBufferIndex + 1) % TX_BATTERY_SAMPLES;
  
  float avgVoltage = txVoltageSum / TX_BATTERY_SAMPLES;
  
  if (avgVoltage <= TX_BATTERY_MIN) return 0;
  if (avgVoltage >= TX_BATTERY_MAX) return 100;
  
  int percent = (int)((avgVoltage - TX_BATTERY_MIN) / (TX_BATTERY_MAX - TX_BATTERY_MIN) * 100.0);
  
  //don't update if change is not greater than 1%
  if (millis() - lastTxUpdate >= txUpdateInterval) 
  {
    lastTxUpdate = millis();
    lastStableTxPercent = percent;
  }
  return (int)lastStableTxPercent;
}

/*
void printBatteryDebug() 
{
  static unsigned long lastDebug = 0;
  
  if (millis() - lastDebug >= serialDebugInterval) 
  {
    //Get TX measurements
    int rawADC = analogRead(TX_BATTERY_PIN);
    float espVoltage = rawADC * 3.3 / 4095.0;
    float rawCalculated = espVoltage * 21.606;
    float correctedVoltage = rawCalculated * TX_CORRECTION_FACTOR;
    int txPercent = calculateTxBatteryPercent();
    
    Serial.println("\n=== BATTERY STATUS ===");
    
    //RX Battery status
    bool rxConnected = telemetryReceived && (millis() - lastTelemetryTime < telemetryTimeout);
    if (rxConnected) 
    {
      Serial.printf("RX: %.2fV → %d%%\n", rxTelemetry.rxBattery, rxBatteryPercent);
    } 
    else 
    {
      Serial.printf("RX: No signal\n");
    }
    
    //TX Battery
    Serial.printf("TX: %.2fV → %d%% (ADC: %d)\n", correctedVoltage, txPercent, rawADC);
    Serial.printf("Filter: %d samples, Update: %dms\n", TX_BATTERY_SAMPLES, txUpdateInterval);
    Serial.println("======================\n");
    
    lastDebug = millis();
  }
}
*/

bool isReceiverConnected() 
{
  return telemetryReceived && (millis() - lastTelemetryTime < telemetryTimeout);
}

void readDigitalToggleSwitches() 
{
  static int lastLightMode = -1;
  static int lastTurnMode = -1;
  
  bool lightPos1 = (digitalRead(LIGHT_POS1_PIN) == LOW);
  bool lightPos3 = (digitalRead(LIGHT_POS3_PIN) == LOW);
  
  if (lightPos1 && !lightPos3) 
  {
    lightMode = 0;
  } 
  else if (!lightPos1 && !lightPos3) 
  {
    lightMode = 1;
  } 
  else if (!lightPos1 && lightPos3) 
  {
    lightMode = 2;
  } 
  else 
  {
    lightMode = 1;
  }
  
  bool turnPos1 = (digitalRead(TURN_POS1_PIN) == LOW);
  bool turnPos3 = (digitalRead(TURN_POS3_PIN) == LOW);
  
  if (turnPos1 && !turnPos3) 
  {
    turnSignalMode = 0;
  } 
  else if (!turnPos1 && !turnPos3) 
  {
    turnSignalMode = 1;
  } 
  else if (!turnPos1 && turnPos3) 
  {
    turnSignalMode = 2;
  } 
  else 
  {
    turnSignalMode = 1;
  }
  
  txData.ledMode = lightMode;
  txData.turnSignal = turnSignalMode;
  
  if (lastLightMode != lightMode || lastTurnMode != turnSignalMode) 
  {
    Serial.printf("Toggles: Light=%d, Turn=%d\n", lightMode, turnSignalMode);
    lastLightMode = lightMode;
    lastTurnMode = turnSignalMode;
  }
}

void setupDigitalToggleSwitches() 
{
  pinMode(LIGHT_POS1_PIN, INPUT_PULLUP);
  pinMode(LIGHT_POS3_PIN, INPUT_PULLUP);
  pinMode(TURN_POS1_PIN, INPUT_PULLUP);
  pinMode(TURN_POS3_PIN, INPUT_PULLUP);
  
  Serial.println("Toggle switches initialized");
}
void setup() 
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==========================================");
  Serial.println("RC TRANSMITTER");
  Serial.println("==========================================");
  
  //init the buffers
  for (int i = 0; i < RX_BATTERY_SAMPLES; i++) 
  {
    rxVoltageBuffer[i] = 0;
    rxVoltageSum += 0;
  }
  
  for (int i = 0; i < TX_BATTERY_SAMPLES; i++) 
  {
    txVoltageBuffer[i] = 0;
    txVoltageSum += 0;
  }
  
  //init LCD
  Wire.begin(21, 22);
  display.begin();
  display.setContrast(255);
  
  //Startup logos
  display.clearBuffer();
  drawAudiQuattroLogo();
  display.sendBuffer();
  delay(2000);
  
  display.clearBuffer();
  display.setFont(u8g2_font_logisoso16_tr);
  display.drawStr(10, 30, "BY");
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(20, 50, "Rock-Robotics");
  display.sendBuffer();
  delay(2000);
  
  Serial.println("LCD ready");
  
  //Init toggles
  setupDigitalToggleSwitches();
  
  //init wifi
  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("ESP failed");
    ESP.restart();
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  //Setup TX battery pin
  pinMode(TX_BATTERY_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  //setup receiver
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
    Serial.println("Peer add failed");
  }
  
  startTime = millis();
  Serial.println("\nSystem ready!");
}

void loop() 
{
  //Read toggles
  readDigitalToggleSwitches();
  
  //Read controls
  int throttleRaw = analogRead(THROTTLE_POT_PIN);
  int steeringRaw = analogRead(STEERING_POT_PIN);
  
  if (abs(throttleRaw - THROTTLE_RAW_CENTER) < THROTTLE_DEADBAND) 
  {
    throttleRaw = THROTTLE_RAW_CENTER;
  }
  if (abs(steeringRaw - STEERING_RAW_CENTER) < STEERING_DEADBAND) 
  {
    steeringRaw = STEERING_RAW_CENTER;
  }
  
  if (throttleRaw <= THROTTLE_RAW_CENTER) 
  {
    txData.throttle = map(throttleRaw, THROTTLE_RAW_FORWARD, THROTTLE_RAW_CENTER, 100, 0);
  } 
  else 
  {
    txData.throttle = map(throttleRaw, THROTTLE_RAW_CENTER, THROTTLE_RAW_REVERSE, 0, -100);
  }
  
  if (steeringRaw >= STEERING_RAW_CENTER) 
  {
    txData.steering = map(steeringRaw, STEERING_RAW_CENTER, STEERING_RAW_RIGHT, 0, -100);
  } 
  else 
  {
    txData.steering = map(steeringRaw, STEERING_RAW_LEFT, STEERING_RAW_CENTER, 100, 0);
  }
  
  txData.throttle = constrain(txData.throttle, -100, 100);
  txData.steering = constrain(txData.steering, -100, 100);
  
  txBatteryPercent = calculateTxBatteryPercent();
  txData.txBattery = readTxBatteryVoltage(); // Use for telemetry
  
  //send data
  txData.timestamp = millis();
  txData.seq_num = sequence++;
  esp_now_send(receiverMAC, (uint8_t *)&txData, sizeof(txData));
  
  //Update display
  if (millis() - lastDisplayUpdate >= displayInterval) 
  {
    lastDisplayUpdate = millis();
    
    unsigned long elapsed = millis() - startTime;
    int hours = (elapsed / 3600000) % 24;
    int minutes = (elapsed / 60000) % 60;
    
    bool rxConnected = isReceiverConnected();
    
    drawDashboard(displayedRPM, displayedSpeed, displayedDistance, rxBatteryPercent, txBatteryPercent, hours, minutes, rxConnected);
  }
  
  //printBatteryDebug();
  
  delay(50);
}

//draw functions
void drawDashboard(int rpm, int speed, float distance, int RxBatteryPercent, int TxBatteryPercent, int hours, int minutes, bool rxConnected) 
{
  display.clearBuffer();
  drawRPMGauge(rpm);
  drawSpeedBox(speed);
  drawDistanceBox(distance);
  drawRxBar(RxBatteryPercent, rxConnected);
  drawTxBar(TxBatteryPercent);
  drawTime(hours, minutes);
  display.sendBuffer();
}

void drawRPMGauge(int rpm) 
{
  int maxRPM = 8000;
  int cx = 32, cy = 45;
  int outerRadius = 30;
  int innerRadius = 24;
  
  float targetAngle = map(constrain(rpm, 0, maxRPM), 0, maxRPM, 225, -45);
  
  for (int angle = 225; angle >= -45; angle -= 3) 
  {
    float rad = angle * PI / 180.0;
    int x1 = cx + outerRadius * cos(rad);
    int y1 = cy - outerRadius * sin(rad);
    int x2 = cx + innerRadius * cos(rad);
    int y2 = cy - innerRadius * sin(rad);
    
    if (angle >= targetAngle) 
    {
      display.drawLine(x1, y1, x2, y2);
      
      if (angle % 30 == 15) 
      {
        int x3 = cx + (outerRadius + 3) * cos(rad);
        int y3 = cy - (outerRadius + 3) * sin(rad);
        display.drawLine(x1, y1, x3, y3);
      }
    }
  }
  
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(2, 8, "RPM");
  
  char rpmStr[10];
  if (rpm < 1000) 
  {
    sprintf(rpmStr, " %d", rpm);
  } 
  else 
  {
    sprintf(rpmStr, "%d", rpm);
  }
  display.drawStr(18, 45, rpmStr);
  display.drawStr(16, 63, "x1000");
  display.drawVLine(64, 0, 64);
}

void drawSpeedBox(int speed) 
{
  int boxX = 66, boxY = 1;
  int boxW = 61, boxH = 20;
  
  display.drawFrame(boxX, boxY, boxW, boxH);
  
  display.setFont(u8g2_font_logisoso16_tr);
  char speedStr[10];
  if (speed < 100) 
  {
    sprintf(speedStr, " %d", speed);
  } 
  else 
  {
    sprintf(speedStr, "%d", speed);
  }
  if (speed < 10) 
  {
    sprintf(speedStr, "  %d", speed);
  }
  display.drawStr(boxX + 1, boxY + 18, speedStr);
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(boxX + 36, boxY + 19, "KM/H");
}

void drawDistanceBox(float distance) 
{
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(66, 31, "TRIP:");
  
  char distStr[15];
  sprintf(distStr, "%.1fkm", distance);
  display.drawStr(94, 31, distStr);
}

void drawRxBar(int RxBatteryPercent, bool connected) 
{
  int barX = 80, barY = 33;
  int barW = 47, barH = 7;
  
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(68, barY + 6, "RX");
  
  display.drawFrame(barX, barY, barW, barH);
  
  if (connected) 
  {
    int fillW = map(constrain(RxBatteryPercent, 0, 100), 0, 100, 0, barW - 2);
    if (fillW > 0) 
    {
      display.drawBox(barX + 1, barY + 1, fillW, barH - 2);
    }
  }
  //just gonna show empty bar if disconnected
}

void drawTxBar(int TxBatteryPercent) 
{
  int barX = 80, barY = 44;
  int barW = 47, barH = 7;
  
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(68, barY + 6, "TX");
  
  display.drawFrame(barX, barY, barW, barH);
  
  int fillW = map(constrain(TxBatteryPercent, 0, 100), 0, 100, 0, barW - 2);
  if (fillW > 0) 
  {
    display.drawBox(barX + 1, barY + 1, fillW, barH - 2);
  }
}

void drawTime(int hours, int minutes) 
{
  display.setFont(u8g2_font_6x10_tr);
  char timeStr[10];
  sprintf(timeStr, "%02d:%02d", hours, minutes);
  display.drawStr(84, 63, timeStr);
}

void drawAudiQuattroLogo() 
{
  int ringRadius = 12;
  int ringThickness = 3;
  int centerY = 20;
  
  int ring1X = 37;
  int ring2X = 55;
  int ring3X = 73;
  int ring4X = 91;
  
  for (int i = 0; i < ringThickness; i++) 
  {
    display.drawCircle(ring1X, centerY, ringRadius + i, U8G2_DRAW_ALL);
    display.drawCircle(ring2X, centerY, ringRadius + i, U8G2_DRAW_ALL);
    display.drawCircle(ring3X, centerY, ringRadius + i, U8G2_DRAW_ALL);
    display.drawCircle(ring4X, centerY, ringRadius + i, U8G2_DRAW_ALL);
  }
  
  display.setFont(u8g2_font_logisoso16_tr);
  display.drawStr(30, 58, "quattro");
}
