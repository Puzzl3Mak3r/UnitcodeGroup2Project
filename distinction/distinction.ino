/*
 * Student Name: Jaymond Martin
 * Student ID: 102579706
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Tasks: Standalone Logic, Dual Hardware Timer ISRs, Hardware-Level SDI-12 TX, NVIC Configuration]
 * - Taha Mohamed Shaik (105910382) [Tasks: SD Card Logging, True Hardware RX Interrupt, Data Averaging]
 * - Chinmayee Sharma (105702631) [Tasks: LCD Dashboard, Pushbutton ISRs, Visual Indicator]
 * 
 * Description: Project Distinction Task - "High-Performance Interrupt Architecture"
 * Upgrades the Edge Data Hub by replacing basic polling with an interrupt-driven 
 * architecture. Utilizes a true hardware interrupt for UART RX detection and dual 
 * hardware timers for independent I2C sensor sampling. Implements ARM NVIC priority 
 * shifting to ensure SDI-12 communication preempts sensor reads, preventing data loss.
 * 
 * Hardware Mapping (Swinburne Arduino Due Board):
 * - BME680 & BH1750 & DS1307 RTC -> I2C (SDA/SCL)
 * - SDI-12 UART Converter -> Serial1 (TX1=18, RX1=19), DIRO -> Pin 8
 * - TFT LCD -> Software SPI (Pins 10, 7, 11, 13)
 * - SD Card -> Software SPI (Pins A3, 12, 11, 13)
 * - Pushbuttons -> Pins 2, 3 (Hardware Interrupts)
 * - Timer3 -> Hardware Interrupt for BH1750 sampling (150ms)
 * - Timer4 -> Hardware Interrupt for BME680 sampling (3000ms)
 * - Comm LED -> Pin 9 (Bargraph LED)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>
#include "RTClib.h"
#include "SdFat.h"
#include <DueTimer.h> 

// --- SENSOR & RTC OBJECTS ---
Adafruit_BME680 bme;
BH1750 lightMeter;
RTC_DS1307 rtc;

// --- HARDWARE PIN DEFINITIONS ---
// TFT LCD Pins
const int TFT_CS   = 10;
const int TFT_RST  = 6; 
const int TFT_DC   = 7; 
const int TFT_SCLK = 13;   
const int TFT_MOSI = 11;   

// Visual Indicator LED
const int COMM_LED = 9;

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
const int DIRO_PIN = 8;
char address = '0';
String commandBuffer = "";
String dataBuffer = "+0.00+0.00+0.00+0.00+0.00";

// --- NON-BLOCKING TRANSMISSION VARIABLES ---
// Utilized to replace delay() and flush() functions, preventing main loop paralysis
int txStage = 0; 
unsigned long txTimer = 0;
int txDuration = 0;
String pendingTxMessage = "";

// --- STANDALONE LOGGING VARIABLES ---
unsigned long lastLogTime = 0;
const unsigned long logInterval = 60000; // 60 seconds
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 3000; // 3 seconds 

// --- SENSOR ISR VARIABLES ---
// Variables modified inside an ISR must be declared volatile
volatile float luxTotal = 0.0;
volatile unsigned long luxSampleCount = 0;

volatile float bmeTemp = 0.0;
volatile float bmePress = 0.0;
volatile float bmeHum = 0.0;
volatile float bmeGas = 0.0;

// Asynchronous BME680 reading variables
volatile uint32_t bmeEndTime = 0;
volatile bool bmeIsReading = false;

// --- DISTINCTION ISR VARIABLES ---
volatile bool triggerLogCycle = false;
volatile bool triggerClearSD = false;

volatile unsigned long lastBtnLogTime = 0;
volatile unsigned long lastBtnClearTime = 0;

volatile bool rxActivity = false;
volatile bool commLedActive = false;
volatile unsigned long lastCommTime = 0;
volatile bool ignoreRX = false;

// --- FUNCTION PROTOTYPES ---
void handleCommand(String cmd);
void triggerSDI12Response(String message);
void handleSDI12TX();
void processIncomingUART();
void executeLogCycle();
String getTimestamp();
String formatSDI12Value(float value);
void initSDCard();
void logToSDCard(String logEntry);
void clearSDCard();
void initLCD();
void updateDashboard();
void luxSamplingISR();
void bmeSamplingISR();
void commISR();
void btnLogISR();
void btnClearISR();
void updateDataBuffer();
void printSDCardContents();

void setup() {
  Serial.begin(9600);
  delay(1000); // Wait for IDE Serial Monitor to connect before booting, but avoid standalone deadlock
  Serial.println("--- High-Performance Sensor Node Booting ---");

  // 1. Initialize SDI-12 UART
  Serial1.begin(1200, SERIAL_7E1);
  pinMode(DIRO_PIN, OUTPUT);
  digitalWrite(DIRO_PIN, HIGH);

  // 2. Initialize Pushbuttons and LEDs
  pinMode(btnLogPin, INPUT_PULLUP);
  pinMode(btnClearPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnLogPin), btnLogISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnClearPin), btnClearISR, FALLING);
  
  pinMode(COMM_LED, OUTPUT);
  digitalWrite(COMM_LED, LOW);

  // Attach True Hardware Interrupt to RX1 (Pin 19).
  // Fires on a FALLING edge, capturing the start bit of incoming UART data.
  // This is kept attached permanently to avoid SAM3X8E PIO register lockups.
  attachInterrupt(digitalPinToInterrupt(19), commISR, FALLING);
  
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
    bme.setGasHeater(320, 150); // 150ms heater profile
  }

  // 4. Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found on I2C bus!");
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 5. Initialize SD Card and LCD
  initSDCard();
  initLCD();

  // 6. Initialize Dual Hardware Timers
  // Note: The project brief suggested a 10ms sampling rate, but the BH1750 operating 
  // in CONTINUOUS_HIGH_RES_MODE physically requires ~120ms to integrate a measurement. 
  // Setting the timer to 10ms would crash the I2C bus. Therefore, 150,000us (150ms) 
  // is the optimal hardware-safe sampling interval.
  Timer3.attachInterrupt(luxSamplingISR);
  Timer3.start(150000); 

  // DueTimer takes arguments in microseconds. 3,000,000us = 3000ms.
  // 3 seconds allows the BME680 gas heater ample time to cycle safely.
  Timer4.attachInterrupt(bmeSamplingISR);
  Timer4.start(3000000); 

  // ==========================================================
  // AUTHOR: Jaymond Martin (102579706)
  // DISTINCTION TASK: NVIC PRIORITY ARCHITECTURE
  // ==========================================================
  // Force the hardware UART (Serial1) to the highest possible priority (0)
  // This guarantees incoming SDI-12 bytes will preempt ANY other task.
  NVIC_SetPriority(USART0_IRQn, 0); 
  
  // Elevate Pin 19 (RX1 / PA10) external interrupt to priority 1
  NVIC_SetPriority(PIOA_IRQn, 1);

  // Demote all Hardware Timers to priority 4. 
  // If the timer is stuck reading I2C, the UART can now safely interrupt the interrupt.
  for (int i = TC0_IRQn; i <= TC8_IRQn; i++) {
    NVIC_SetPriority((IRQn_Type)i, 4); 
  }

  Serial.println("System Ready. Distinction Interrupt Architecture Active.");
  
  // Dump the file contents to verify previous logs were saved
  printSDCardContents();
}

void loop() {
  // ==========================================================
  // ASYNCHRONOUS BME680 DATA COLLECTION
  // ==========================================================
  if (bmeIsReading && millis() >= bmeEndTime) {
    if (bme.endReading()) {
      noInterrupts();
      bmeTemp = bme.temperature;
      bmePress = bme.pressure / 100.0;
      bmeHum = bme.humidity;
      bmeGas = bme.gas_resistance / 1000.0;
      interrupts();
    }
    bmeIsReading = false;
  }

  // ==========================================================
  // ASYNCHRONOUS UART TRANSMISSION & RECEPTION
  // ==========================================================
  
  if (txStage > 0) {
    handleSDI12TX();
  } else {
    // Process incoming UART data triggered by ISR or background buffer
    if (rxActivity || Serial1.available() > 0) {
      processIncomingUART();
      rxActivity = false; // Safely clear the flag ONCE after processing
    }

    // Process hardware button interrupts
    if (triggerLogCycle) {
      triggerLogCycle = false;
      executeLogCycle();
    }

    if (triggerClearSD) {
      triggerClearSD = false;
      clearSDCard();
    }

    // Turn off the Communication LED 50ms after the last UART RX interrupt
    if (commLedActive && (millis() - lastCommTime > 50)) {
      digitalWrite(COMM_LED, LOW);
      commLedActive = false;
    }

    // Non-blocking 3-second timer for real-time dashboard updates
    if (millis() - lastDisplayTime >= displayInterval) {
      lastDisplayTime = millis();
      updateDataBuffer(); 
      updateDashboard(); 
    }

    // Non-blocking 60-second timer for automatic SD Card logging
    if (millis() - lastLogTime >= logInterval) {
      lastLogTime = millis();
      executeLogCycle();
    }
  }
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Distinction - Dual Hardware Timer ISRs
// ==========================================================

// ISR for Light Sensor (Fires every 150ms)
void luxSamplingISR() {
  // Re-enable global interrupts inside this ISR. 
  // This allows the I2C (Wire) library to function without deadlocking the Due.
  interrupts(); 
  
  float currentLux = lightMeter.readLightLevel();
  luxTotal += currentLux;
  luxSampleCount++;
}

// ISR for Environment Sensor (Fires every 3000ms)
void bmeSamplingISR() {
  interrupts(); 
  
  // Triggers the asynchronous reading process without utilizing delay()
  if (!bmeIsReading) {
    bmeEndTime = bme.beginReading();
    if (bmeEndTime != 0) {
      bmeIsReading = true;
    }
  }
}

// Compiles the volatile floats from the ISRs into a safe, formatted string
void updateDataBuffer() {
  float safeTemp, safePress, safeHum, safeGas, safeLux;

  // Briefly suspend interrupts to copy the variables, preventing memory 
  // corruption if the ISR fires exactly while the variables are being read.
  noInterrupts();
  safeTemp = bmeTemp;
  safePress = bmePress;
  safeHum = bmeHum;
  safeGas = bmeGas;
  
  if (luxSampleCount > 0) {
    safeLux = luxTotal / luxSampleCount;
  } else {
    safeLux = 0.0; 
  }
  interrupts();

  dataBuffer = "";
  dataBuffer += formatSDI12Value(safeTemp);
  dataBuffer += formatSDI12Value(safePress);
  dataBuffer += formatSDI12Value(safeHum);
  dataBuffer += formatSDI12Value(safeGas);
  dataBuffer += formatSDI12Value(safeLux);
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Pass & Distinction - Basic If/Else Command Handler
// ==========================================================

void handleCommand(String cmd) {
  cmd.trim();

  // Address Query
  if (cmd == "?!") {
    triggerSDI12Response(String(address));
  } 
  // Change Address
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'A' && cmd[3] == '!') {
    address = cmd[2]; 
    triggerSDI12Response(String(address));
  } 
  // Start Measurement
  else if (cmd == String(address) + "M!") {
    triggerSDI12Response(String(address) + "0035");
  } 
  // Send Data
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'D' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      updateDataBuffer(); 
      triggerSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      triggerSDI12Response(String(address)); 
    }
  } 
  // Continuous Measurement
  else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'R' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      updateDataBuffer();
      triggerSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      triggerSDI12Response(String(address)); 
    }
  }
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Distinction - Hardware-Level SDI-12 TX FSM
// ==========================================================

// Initiates asynchronous transmission using boolean flags and millis()
void triggerSDI12Response(String message) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Sending Response: "); 
  Serial.println(message);
  
  pendingTxMessage = message + "\r\n";
  
  // Calculate max transmission time: ~8.33ms per char at 1200 baud. 
  // Utilizing 10ms per char guarantees the software buffer pushes it out.
  txDuration = pendingTxMessage.length() * 10; 
  
  ignoreRX = true; 
  
  // Phase 1: Wait for Master to release the line. 
  // Do NOT pull DIRO_PIN low yet to prevent bus collisions.
  txTimer = millis();
  txStage = 1; 
}

// Evaluates the transmission sequence without utilizing blocking delay() or flush()
void handleSDI12TX() {
  if (txStage == 1) {
    // SDI-12 Specification: Slave must wait at least 8.33ms 
    // after receiving a command before driving the bus.
    if (millis() - txTimer >= 10) {
      digitalWrite(DIRO_PIN, LOW); // Claim the bus
      txTimer = millis();
      txStage = 2;
    }
  } 
  else if (txStage == 2) {
    // Allow the physical RS485 transceiver 2ms to open its TX gates 
    // before pushing bits into the hardware buffer.
    if (millis() - txTimer >= 2) {
      Serial1.print(pendingTxMessage);
      txTimer = millis();
      txStage = 3;
    }
  } 
  else if (txStage == 3) {
    // Wait for the calculated duration so the software buffer empties
    if (millis() - txTimer >= txDuration) {
      // Confirm the silicon register has pushed the final stop bit
      if (USART0->US_CSR & US_CSR_TXEMPTY) {
        digitalWrite(DIRO_PIN, HIGH); // Revert to RX mode
        txTimer = millis();
        txStage = 4;
      }
    }
  } 
  else if (txStage == 4) {
    // Wait 25ms for line capacitance to drain and flush bounce noise
    if (millis() - txTimer >= 25) { 
      while(Serial1.available() > 0) {
        Serial1.read(); 
      }
      ignoreRX = false; 
      txStage = 0; 
    }
  }
}

// ==========================================================
// AUTHOR: Taha Mohamed Shaik (105910382)
// TASK: Distinction - True Hardware RX Interrupt & Parsing
// ==========================================================

// Fires instantly when voltage changes on the RX line (Pin 19)
void commISR() {
  if (ignoreRX) return; // Instantly exit if switching TX/RX modes
  
  rxActivity = true; 
  digitalWrite(COMM_LED, HIGH);
  commLedActive = true;
  lastCommTime = millis(); 
}

void processIncomingUART() {
  while (Serial1.available() > 0) {
    char incomingByte = Serial1.read();

    // 1. Trap the SDI-12 Break (0x00) or UART framing errors (0xFF)
    if (incomingByte == 0 || incomingByte == 255) {
      commandBuffer = ""; // Instantly wipe any old noise
      continue; // Skip the rest of the loop for this byte
    }

    // DEBUG: Print valid bytes
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] [DEBUG RX] Byte: ");
    Serial.println(incomingByte);

    // 2. Ignore CRLF, append everything else
    if (incomingByte != '\r' && incomingByte != '\n') {
      commandBuffer += incomingByte;
    }

    // 3. Process command on termination character
    if (incomingByte == '!') {
      Serial.print("[");
      Serial.print(millis());
      Serial.print("] Received Raw Buffer: ");
      Serial.println(commandBuffer);
      
      // Right-to-Left tail inspection algorithm to scrub electrical noise 
      String cleanCmd = "";
      int len = commandBuffer.length();

      // Pattern 1: Address Query (?!) - Length 2
      if (len >= 2 && commandBuffer.substring(len - 2) == "?!") {
        cleanCmd = "?!";
      } 
      // Pattern 2: Start Measurement (aM!) - Length 3
      else if (len >= 3 && commandBuffer.endsWith("M!") && commandBuffer[len - 3] == address) {
        cleanCmd = commandBuffer.substring(len - 3);
      } 
      // Pattern 3: Three-character prefixed commands (aAb!, aD0!, aR0!) - Length 4
      else if (len >= 4 && commandBuffer.endsWith("!") && commandBuffer[len - 4] == address) {
        char cmdChar = commandBuffer[len - 3];
        if (cmdChar == 'A' || cmdChar == 'D' || cmdChar == 'R') {
          cleanCmd = commandBuffer.substring(len - 4);
        }
      }

      if (cleanCmd != "") {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] Parsed Clean Command: ");
        Serial.println(cleanCmd);
        handleCommand(cleanCmd);
      } else {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] Ignored Noise Buffer. Active Address: ");
        Serial.println(address);
      }
      
      commandBuffer = ""; // Reset buffer after execution
    }
    
    // Failsafe: Prevent memory overflow if the line gets stuck with noise
    if (commandBuffer.length() > 30) {
      commandBuffer = "";
    }
  }
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Credit - Core Logging & Timestamp Generation
// ==========================================================

String formatSDI12Value(float value) {
  if (value >= 0.0) {
    return "+" + String(value, 2);
  } else {
    return String(value, 2); // Negative values already include the '-' sign
  }
}

void executeLogCycle() {
  updateDataBuffer();

  String timestamp = getTimestamp();
  String logEntry = timestamp + ", " + dataBuffer;

  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Auto-Log Triggered: ");
  Serial.println(logEntry);

  logToSDCard(logEntry);
  updateDashboard();

  // Reset the averaging variables safely
  noInterrupts();
  luxTotal = 0.0;
  luxSampleCount = 0;
  interrupts();
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
// TASK: Credit - LCD Dashboard & Pushbutton ISRs
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
  float safeTemp, safeHum, safePress, safeLux;
  
  // Safely copy volatile variables for display
  noInterrupts();
  safeTemp = bmeTemp;
  safeHum = bmeHum;
  safePress = bmePress;
  safeLux = (luxSampleCount > 0) ? (luxTotal / luxSampleCount) : 0.0;
  interrupts();

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

  tft.setCursor(65, 30);
  tft.print(safeTemp);
  tft.print(" C   ");

  tft.setCursor(65, 50);
  tft.print(safeHum);
  tft.print(" %   ");

  tft.setCursor(65, 70);
  tft.print(safePress);
  tft.print(" hPa   ");
  
  tft.setCursor(65, 90);
  tft.print(safeLux); 
  tft.print(" Lux   ");
}

// Hardware Interrupt for Manual Log Button
void btnLogISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastBtnLogTime > 200) { 
    triggerLogCycle = true;
    lastBtnLogTime = currentTime;
  }
}

// Hardware Interrupt for Clear SD Button
void btnClearISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastBtnClearTime > 200) { 
    triggerClearSD = true;
    lastBtnClearTime = currentTime;
  }
}