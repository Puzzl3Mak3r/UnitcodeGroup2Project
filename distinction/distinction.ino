/*
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Tasks: Standalone Logic, RTC, Timer ISR Routing]
 * - Taha Mohamed Shaik (105910382) [Tasks: SD Card Logging, UART RX Interrupt, Data Averaging]
 * - Chinmayee Sharma (105702631) [Tasks: LCD Dashboard, Pushbuttons, Visual Indicator]
 * 
 * Description: Project Distinction Task - "High-Performance Interrupt Architecture"
 * Upgrades the Edge Data Hub by replacing basic polling with an interrupt-driven 
 * architecture. Shifts UART communication to an RX interrupt, samples the light 
 * sensor every 10ms via a hardware timer to calculate accurate averages, and 
 * toggles a visual indicator during active communication.
 * 
 * Hardware Mapping (Swinburne Arduino Due Board):
 * - BME680 & BH1750 & DS1307 RTC -> I2C (SDA/SCL)
 * - SDI-12 UART Converter -> Serial1 (TX1=18, RX1=19), DIRO -> Pin 7
 * - TFT LCD -> Software SPI (Pins 10, 7, 11, 13)
 * - SD Card -> Software SPI (Pins A3, 12, 11, 13)
 * - Pushbuttons -> Pins 2, 3
 * - Timer3 -> Hardware Interrupt for 10ms sampling
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include "RTClib.h"
#include "SdFat.h"
#include <DueTimer.h> // Required for SAM3X8E Hardware Timers

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

// --- DISTINCTION ISR VARIABLES ---
// Variables modified inside an ISR must be volatile so they are fetched directly from RAM
volatile bool takeFastSample = false;

// Function prototypes (Pass & Credit)
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

// Function prototypes (Distinction)
void samplingISR();
void executeFastSampling();
void accumulateLuxData(float currentLux);
void calculateAverages();
void toggleCommunicationLED();

void setup() {
  Serial.begin(9600);
  Serial.println("--- High-Performance Sensor Node Booting ---");

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

  // 4. Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found on I2C bus!");
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 5. Initialize Teammate Modules (Credit Task)
  initSDCard();
  initLCD();

  // 6. Initialize Hardware Timer (Distinction Task)
  // Attach the ISR and start the timer at 10,000 microseconds (10ms)
  Timer3.attachInterrupt(samplingISR);
  Timer3.start(10000);

  Serial.println("System Ready. Interrupt Architecture Active.");
}

void loop() {
  // ==========================================================
  // DISTINCTION TASK: DEFERRED ISR PROCESSING
  // ==========================================================
  // I2C communication inside a Timer ISR can crash the Arduino Due. 
  // The ISR sets a flag, and the actual reading happens safely here.
  if (takeFastSample) {
    takeFastSample = false;
    executeFastSampling();
  }

  // NOTE: UART polling (Serial1.available) has been removed from the main loop.
  // It is now handled by the serialEvent1() interrupt routine at the bottom of the file.

  // ==========================================================
  // STANDALONE TIMER LOGIC (Credit Task)
  // ==========================================================
  // Non-blocking 60-second timer for automatic logging
  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();
    executeLogCycle();
  }

  // ==========================================================
  // CONFIG MENU POLLING (Credit Task)
  // ==========================================================
  checkPushbuttons();
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Distinction - Hardware Timer Routing & Safe Sampling
// ==========================================================

// Hardware Timer Interrupt Service Routine (Fires every 10ms)
void samplingISR() {
  takeFastSample = true; // Signal the main loop to read the sensor safely
  toggleCommunicationLED(); // Trigger the visual indicator directly
}

// Safely executes the 10ms sensor read outside the ISR
void executeFastSampling() {
  float currentLux = lightMeter.readLightLevel();
  
  // Pass the raw 10ms reading to the averaging logic
  accumulateLuxData(currentLux); 
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Credit - Core Logging & Timestamp Generation
// ==========================================================
void executeLogCycle() {
  readSensors();
  
  // Trigger the math calculation to append the averaged lux before logging
  calculateAverages();

  String timestamp = getTimestamp();
  String logEntry = timestamp + ", " + dataBuffer;

  Serial.print("Auto-Log Triggered: ");
  Serial.println(logEntry);

  logToSDCard(logEntry);
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
// TASK: Pass - Read Sensors & SDI-12 Command Parser
// ==========================================================
void readSensors() {
  if (!bme.performReading()) return;
  
  // The lux value will be appended by the calculateAverages() function.
  // This formats the BME680 data.
  dataBuffer = ""; 
  dataBuffer += "+"; dataBuffer += String(bme.temperature, 2);
  dataBuffer += "+"; dataBuffer += String(bme.pressure / 100.0, 2);
  dataBuffer += "+"; dataBuffer += String(bme.humidity, 2);
  dataBuffer += "+"; dataBuffer += String(bme.gas_resistance / 1000.0, 2);
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
    calculateAverages(); // Ensure the latest average is ready for the SDI-12 request
    sendSDI12Response(String(address) + "0035");
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'D' && cmd[3] == '!') {
    if (cmd[2] == '0') sendSDI12Response(String(address) + dataBuffer);
    else if (cmd[2] > '0' && cmd[2] <= '9') sendSDI12Response(String(address)); 
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'R' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      readSensors();
      calculateAverages();
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
// AUTHOR: Taha Mohamed Shaik (105910382)
// TASK: Distinction - UART RX Interrupt & Data Averaging
// ==========================================================

// serialEvent1() acts as an interrupt routine that fires automatically 
// whenever data arrives on the Serial1 RX pin, replacing the main loop polling.
void serialEvent1() {
  // TODO: Taha - Move the UART reading logic from the Pass task into here.
  // Read the incoming byte, append it to the 'command' string, and call 
  // handleCommand() when the '!' terminator is received.
}

void accumulateLuxData(float currentLux) {
  // TODO: Taha - Add the currentLux to a running total and increment a sample counter.
  // This function is called safely from the main loop every 10ms.
}

void calculateAverages() {
  // TODO: Taha - Calculate the average lux from the accumulated data.
  // Reset the running total and counter for the next cycle.
  // Append the averaged lux value to the global 'dataBuffer' string.
  // Example: dataBuffer += "+" + String(averagedLux, 2);
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
  // Note: This will need to be updated to display the averaged lux once calculateAverages() is finished
  tft.print(lightMeter.readLightLevel()); 
  tft.print(" Lux");
}

void checkPushbuttons() {
  int btnLogState = digitalRead(btnLogPin);
  int btnClearState = digitalRead(btnClearPin);

  if (btnLogState == LOW && btnLogLastState == HIGH) {
    executeLogCycle();
    delay(50); 
  }
  btnLogLastState = btnLogState;

  if (btnClearState == LOW && btnClearLastState == HIGH) {
    clearSDCard();
    delay(50); 
  }
  btnClearLastState = btnClearState;
}

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// TASK: Distinction - Visual Indicator
// ==========================================================
void toggleCommunicationLED() {
  // TODO: Chinmayee - Write the logic to toggle an LED on the board.
  // Note: This function runs directly inside the hardware ISR every 10ms.
  // Keep the code extremely short and fast (e.g., digitalWrite).
  // You will also need to define the LED pin at the top of the file and set it as an OUTPUT in setup().
}