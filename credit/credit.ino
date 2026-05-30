/*
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Task: Standalone Logic, RTC & Timestamps]
 * - Taha Mohamed Shaik (105910382) [Task: SD Card File Management, UART Polling]
 * - Chinmayee Sharma (105702631) [Task: LCD Dashboard & Pushbuttons, Command Parser]
 * 
 * Description: Project Credit Task - The "Edge Data Hub"
 * Transforms the compliant sensor node into a standalone logger. 
 * Automatically logs data to an SD card every 60 seconds with an RTC timestamp.
 * Features a real-time LCD dashboard and pushbutton configuration menu.
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
const int btnLogPin = 2;   
const int btnClearPin = 3; 

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
#define DIRO_PIN 8
char address = '0';
String command = "";
String dataBuffer = "+0.00+0.00+0.00+0.00+0.00";

// --- STANDALONE LOGGING & DISPLAY VARIABLES ---
unsigned long lastLogTime = 0;
const unsigned long logInterval = 60000; // 60 seconds in milliseconds

unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 3000; // 3 seconds (Allows BME680 gas heater to cool down)

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
void printSDCardContents();

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for IDE Serial Monitor to connect before booting
  Serial.println("--- Edge Data Hub Booting ---");
  Serial.println("Test1");

  // 1. Initialize SDI-12 UART
  Serial1.begin(1200, SERIAL_7E1);
  pinMode(DIRO_PIN, OUTPUT);
  digitalWrite(DIRO_PIN, HIGH);

  // 2. Initialize Pushbuttons with internal pull-ups 
  pinMode(btnLogPin, INPUT_PULLUP);
  pinMode(btnClearPin, INPUT_PULLUP);
  
  // 3. Initialize SD Card
  initSDCard();
  
  // 4. Initialize I2C Bus & Sensors
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

  // 5. Initialize RTC (Jaymond's Task)
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found on I2C bus!");
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 6. Initialize LCD
  initLCD();

  // Force an initial reading and dashboard update so the screen isn't blank at boot
  readSensors();
  updateDashboard();

  Serial.println("System Ready. Standalone logging active.");
  
  // Dump the file contents to verify previous logs were saved
  printSDCardContents();
}


void loop() {
  // ==========================================================
  // AUTHOR: Taha Mohamed Shaik (105910382)
  // TASK: Check Serial monitor constantly (UART Polling)
  // ==========================================================
  
  // Use 'while' instead of 'if' to ensure the buffer is completely emptied,
  // preventing dropped characters when the loop is slowed down by the LCD.
  while (Serial1.available() > 0) {
    int incomingByte = Serial1.read();
    
    // SDI-12 commands ALWAYS terminate with an exclamation mark '!' (ASCII 33)
    if (incomingByte == 33) {
      Serial.print("Received Command: ");
      Serial.println(command + "!");
      
      handleCommand(command + "!");
      command = ""; 
    } else {
      // Ignore start bits (0) and append valid characters to the buffer
      if (incomingByte != 0 && incomingByte != '\r' && incomingByte != '\n') {
        command += char(incomingByte);
      }
    }
  }

  // ==========================================================
  // AUTHOR: Jaymond Martin (102579706)
  // TASK: Standalone Timer Logic
  // ==========================================================
  
  // Non-blocking 3-second timer for real-time dashboard updates
  if (millis() - lastDisplayTime >= displayInterval) {
    lastDisplayTime = millis();
    readSensors();     
    updateDashboard(); 
  }

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
  // 1. Update the sensor data buffer to ensure the log is perfectly synced
  readSensors();

  // 2. Generate the timestamp
  String timestamp = getTimestamp();

  // 3. Construct the CSV row
  String logEntry = timestamp + ", " + dataBuffer;

  // Print to Serial for debugging
  Serial.print("Auto-Log Triggered: ");
  Serial.println(logEntry);

  // 4. Pass the data to SD Card function
  logToSDCard(logEntry);
  
  // 5. Force a dashboard update so the UI matches the exact logged value
  updateDashboard();
}

String getTimestamp() {
  DateTime now = rtc.now();
  char timeBuffer[25];
  
  sprintf(timeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
          
  return String(timeBuffer);
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Pass - Read Sensors
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

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// TASK: Pass - Command handling look at table (SDI-12 Parser)
// ==========================================================
void handleCommand(String cmd) {
  cmd.trim();
  
  // 1. Address Query: ?!
  if (cmd == "?!") {
    sendSDI12Response(String(address));
  } 
  // 2. Change Address: aAb!
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'A' && cmd[3] == '!') {
    address = cmd[2]; 
    sendSDI12Response(String(address));
  } 
  // 3. Start Measurement: aM!
  else if (cmd == String(address) + "M!") {
    // Respond instantly to prevent SDI-12 master timeout. 
    // Data is already kept fresh by the 3-second display timer.
    sendSDI12Response(String(address) + "0035");
  } 
  // 4. Send Data: aD0! to aD9!
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'D' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      sendSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      sendSDI12Response(String(address)); 
    }
  } 
  // 5. Continuous Measurement: aR0! to aR9!
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'R' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      sendSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      sendSDI12Response(String(address)); 
    }
  }
}

void sendSDI12Response(String message) {
  Serial.print("Sending Response: "); 
  Serial.println(message);
  
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
// TASK: Credit - SD Card File Management
// ==========================================================
void initSDCard() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH); 
  
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Error: SD card initialization failed!");
    sd.initErrorHalt();
  } else {
    Serial.println("SD Card initialized.");
    
    if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_APPEND)) {
      logFile.println("Timestamp, SDI-12_Buffer(+Temp+Pressure+Humidity+Gas+Lux)");
      logFile.close();
      Serial.println("Created/Opened LOG.CSV");
    }
  }
}

void logToSDCard(String logEntry) {
  if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_APPEND)) {
    logFile.println(logEntry);
    logFile.close();
  } else {
    Serial.println("Error: Failed to open LOG.CSV for appending!");
  }
}

void clearSDCard() {
  if (logFile.open("LOG.CSV", O_WRONLY | O_CREAT | O_TRUNC)) {
    logFile.println("Timestamp, SDI-12_Buffer(+Temp+Pressure+Humidity+Gas+Lux)");
    logFile.close();
    Serial.println("LOG.CSV cleared and reset.");
  } else {
    Serial.println("Error: Failed to clear LOG.CSV!");
  }
}

// ==========================================================
// DEBUG HELPER: Dump SD Card contents to Serial Monitor
// ==========================================================
void printSDCardContents() {
  Serial.println("\n--- Reading LOG.CSV ---");
  
  // Open the file in read-only mode
  if (logFile.open("LOG.CSV", O_READ)) {
    // Read and print every character until the end of the file
    while (logFile.available()) {
      Serial.write(logFile.read());
    }
    logFile.close();
    Serial.println("\n--- End of File ---\n");
  } else {
    Serial.println("Error: Could not open LOG.CSV for reading.");
  }
}

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// TASK: Credit - LCD Dashboard & Pushbuttons
// ==========================================================
void initLCD() {
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);           
  tft.fillScreen(ST77XX_BLACK); 

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
  tft.setTextSize(1);
  
  // Using the background color parameter (ST77XX_BLACK) forces the text 
  // to overwrite the old pixels, eliminating the need for a slow fillRect()
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

  // Padding with spaces at the end ensures that if the new number is shorter 
  // than the old number, the leftover trailing characters are erased.
  tft.setCursor(65, 30);
  tft.print(bme.temperature);
  tft.print(" C   ");

  tft.setCursor(65, 50);
  tft.print(bme.humidity);
  tft.print(" %   ");

  tft.setCursor(65, 70);
  tft.print(bme.pressure / 100.0);
  tft.print(" hPa   ");
  
  tft.setCursor(65, 90);
  tft.print(lightMeter.readLightLevel());
  tft.print(" Lux   ");
}

void checkPushbuttons() {
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