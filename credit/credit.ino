/*
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Task: Standalone Logic, RTC & Timestamps]
 * - Taha Mohamed Shaik (105910382) [Task: SD Card File Management]
 * - Chinmayee Sharma (105702631) [Task: LCD Dashboard & Pushbuttons]
 * 
 * Description: Project Credit Task - The "Edge Data Hub"
 * Transforms the compliant sensor node into a standalone logger. 
 * Automatically logs data to an SD card every 60 seconds with an RTC timestamp.
 * Features an LCD dashboard and pushbutton configuration menu.
 * 
 * Hardware Mapping (Swinburne Arduino Due Board):
 * - BME680 & BH1750 & DS1307 RTC -> I2C (SDA/SCL)
 * - SDI-12 UART Converter -> Serial1 (TX1=18, RX1=19), DIRO -> Pin 7
 * - TFT LCD -> Software SPI (Pins 10, 7, 11, 13)
 * - SD Card -> Software SPI (Pins A3, 12, 11, 13)
 * - Pushbuttons -> Pins 2, 3
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include "RTClib.h"
#include "SdFat.h"

// --- SENSOR & RTC OBJECTS ---
Adafruit_BME680 bme;
BH1750 lightMeter;
RTC_DS1307 rtc;

// --- HARDWARE PIN DEFINITIONS ---
// TFT LCD Pins
#define TFT_CS    10
#define TFT_RST   6 
#define TFT_DC    7 
#define TFT_SCLK  13   
#define TFT_MOSI  11   

// Pushbutton Pins
const int btnLogPin = 2;   // Manually trigger a log
const int btnClearPin = 3; // Clear the SD card

// SD Card Pins
const uint8_t SD_CS_PIN = A3;
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN  = 13;

// --- OBJECT INITIALIZATION ---
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Setup SD Software SPI Driver utilizing the SdFat library
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
SdFs sd;
FsFile logFile;

// --- SDI-12 STATE VARIABLES ---
#define DIRO_PIN 7 
char address = '0';
String command = "";
String dataBuffer = "+0.00+0.00+0.00+0.00+0.00";

// --- STANDALONE LOGGING VARIABLES ---
unsigned long lastLogTime = 0;
const unsigned long logInterval = 60000; // 60 seconds in milliseconds

// --- BUTTON STATE VARIABLES ---
int btnLogLastState = HIGH;
int btnClearLastState = HIGH;

// Function prototypes
void handleCommand(String cmd);
void sendSDI12Response(String message);
void readSensors();
void executeLogCycle();
String getTimestamp();
void initSDCard();
void logToSDCard(String logEntry);
void clearSDCard();
void initLCD();
void updateDashboard();
void checkPushbuttons();

void setup() {
  Serial.begin(9600);
  Serial.println("--- Edge Data Hub Booting ---");

  // 1. Initialize SDI-12 UART
  Serial1.begin(1200, SERIAL_7E1);
  pinMode(DIRO_PIN, OUTPUT);
  digitalWrite(DIRO_PIN, HIGH);

  // 2. Initialize Pushbuttons with internal pull-ups
  pinMode(btnLogPin, INPUT_PULLUP);
  pinMode(btnClearPin, INPUT_PULLUP);
  
  // 3. Initialize I2C Bus & Sensors
  Wire.begin();

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Error: BH1750 initialization failed!");
  }
  if (!bme.begin(0x76)) {
    Serial.println("Error: BME680 initialization failed!");
  } else {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
  }

  // 4. Initialize RTC (Jaymond's Task)
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found on I2C bus!");
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 5. Initialize Teammate Modules
  initSDCard();
  initLCD();

  Serial.println("System Ready. Standalone logging active.");
}

void loop() {
  // ==========================================================
  // SDI-12 POLLING (Legacy Pass Task Logic)
  // ==========================================================
  if (Serial1.available()) {
    int incomingByte = Serial1.read();
    if (incomingByte == 33) {
      handleCommand(command + "!");
      command = ""; 
    } else {
      if (incomingByte != 0 && incomingByte != '\r' && incomingByte != '\n') {
        command += char(incomingByte);
      }
    }
  }

  // ==========================================================
  // AUTHOR: Jaymond Martin (102579706)
  // TASK: Standalone Timer Logic
  // ==========================================================
  // Non-blocking 60-second timer for automatic logging
  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();
    executeLogCycle();
  }

  // ==========================================================
  // AUTHOR: Chinmayee Sharma (105702631)
  // TASK: Config Menu Polling
  // ==========================================================
  checkPushbuttons();
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Core Logging & Timestamp Generation
// ==========================================================
void executeLogCycle() {
  // 1. Update the sensor data buffer
  readSensors();

  // 2. Generate the timestamp
  String timestamp = getTimestamp();

  // 3. Construct the CSV row (e.g., "2026-05-25 18:15:00, +25.34+1011.24...")
  String logEntry = timestamp + ", " + dataBuffer;

  // Print to Serial for debugging
  Serial.print("Auto-Log Triggered: ");
  Serial.println(logEntry);

  // 4. Pass the data to SD Card function and LCD function
  logToSDCard(logEntry);
  updateDashboard();
}

String getTimestamp() {
  DateTime now = rtc.now();
  char timeBuffer[25];
  
  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
          
  return String(timeBuffer);
}

// ==========================================================
// LEGACY PASS TASK FUNCTIONS (Sensors & SDI-12)
// ==========================================================
void readSensors() {
  if (!bme.performReading()) return;
  float lux = lightMeter.readLightLevel();
  
  dataBuffer = ""; 
  dataBuffer += "+"; dataBuffer += String(bme.temperature, 2);
  dataBuffer += "+"; dataBuffer += String(bme.pressure / 100.0, 2);
  dataBuffer += "+"; dataBuffer += String(bme.humidity, 2);
  dataBuffer += "+"; dataBuffer += String(bme.gas_resistance / 1000.0, 2);
  dataBuffer += "+"; dataBuffer += String(lux, 2);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "?!") {
    sendSDI12Response(String(address));
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'A' && cmd[3] == '!') {
    address = cmd[2]; 
    sendSDI12Response(String(address));
  } else if (cmd == String(address) + "M!") {
    readSensors();
    sendSDI12Response(String(address) + "0035");
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'D' && cmd[3] == '!') {
    if (cmd[2] == '0') sendSDI12Response(String(address) + dataBuffer);
    else if (cmd[2] > '0' && cmd[2] <= '9') sendSDI12Response(String(address)); 
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'R' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      readSensors();
      sendSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') sendSDI12Response(String(address)); 
  }
}

void sendSDI12Response(String message) {
  digitalWrite(DIRO_PIN, LOW); 
  delay(100); 
  Serial1.print(message + "\r\n");
  Serial1.flush(); 
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO_PIN, HIGH); 
}

// ==========================================================
// AUTHOR: Taha Mohamed Shaik (105910382)
// TASK: SD Card File Management
// ==========================================================
void initSDCard() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH); 
  
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Error: SD card initialization failed!");
    sd.initErrorHalt();
  } else {
    Serial.println("SD Card initialized.");
    
    // Create or append to the telemetry log file
    if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_APPEND)) {
      logFile.println("Timestamp, SDI-12_Buffer(+Temp+Pressure+Humidity+Gas+Lux)");
      logFile.close();
      Serial.println("Created/Opened LOG.CSV");
    }
  }
}

void logToSDCard(String logEntry) {
  // Open the CSV file, append the logEntry string, and close it
  if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_APPEND)) {
    logFile.println(logEntry);
    logFile.close();
  } else {
    Serial.println("Error: Failed to open LOG.CSV for appending!");
  }
}

void clearSDCard() {
  // Overwrite the CSV file by opening with O_TRUNC to clear memory,
  // then safely rewrite the header row.
  if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_TRUNC)) {
    logFile.println("Timestamp, SDI-12_Buffer(+Temp+Pressure+Humidity+Gas+Lux)");
    logFile.close();
    Serial.println("LOG.CSV cleared and reset.");
  } else {
    Serial.println("Error: Failed to clear LOG.CSV!");
  }
}

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// TASK: LCD Dashboard & Pushbuttons
// ==========================================================
void initLCD() {
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);           // Rotate to landscape orientation (160x128 pixels)
  tft.fillScreen(ST77XX_BLACK); // Clear display memory

  // Static UI labels
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 10);
  tft.print("EDGE DATA HUB");

  tft.setCursor(5, 30);
  tft.print("Temp:");

  tft.setCursor(5, 50);
  tft.print("Humidity:");

  tft.setCursor(5, 70);
  tft.print("Pressure:");

  tft.setCursor(5, 90);
  tft.print("Light:");
}

void updateDashboard() {
  // Clear only the data area to prevent screen flickering
  tft.fillRect(65, 25, 95, 80, ST77XX_BLACK); 

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);

  tft.setCursor(65, 30);
  tft.print(bme.temperature);
  tft.print(" C");

  tft.setCursor(65, 50);
  tft.print(bme.humidity);
  tft.print(" %");

  tft.setCursor(65, 70);
  tft.print(bme.pressure / 100.0);
  tft.print(" hPa");
  
  tft.setCursor(65, 90);
  tft.print(lightMeter.readLightLevel());
  tft.print(" Lux");
}

void checkPushbuttons() {
  // Read the current physical state of the buttons
  int btnLogState = digitalRead(btnLogPin);
  int btnClearState = digitalRead(btnClearPin);

  // Button 1: Manual Log Trigger (Falling Edge Detection)
  if (btnLogState == LOW && btnLogLastState == HIGH) {
    executeLogCycle();
    delay(50); // Software debounce
  }
  btnLogLastState = btnLogState;

  // Button 2: Clear SD Card (Falling Edge Detection)
  if (btnClearState == LOW && btnClearLastState == HIGH) {
    clearSDCard();
    delay(50); // Software debounce
  }
  btnClearLastState = btnClearState;
}