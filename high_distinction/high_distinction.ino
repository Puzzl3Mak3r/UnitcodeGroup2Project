/*
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Tasks: Standalone Logic, Dual Hardware Timer ISRs, Hardware-Level SDI-12 TX, NVIC Configuration, Watchdog Timer]
 * - Taha Mohamed Shaik (105910382) [Tasks: SD Card Logging, True Hardware RX Interrupt, Data Averaging, Custom Improvement (TBD)]
 * - Chinmayee Sharma (105702631) [Tasks: LCD Dashboard, Pushbutton ISRs, Visual Indicator, FSM Command Parser]
 * 
 * Description: Project High Distinction Task - "Advanced Event-Driven Node"
 * Upgrades the Edge Data Hub into a professional-grade, fail-safe system. 
 * Implements a Watchdog Timer (WDT) to automatically reset the board if I2C 
 * sensors hang. Replaces basic string parsing with a robust Finite State Machine 
 * (FSM) to gracefully handle corrupted SDI-12 commands.
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
#define TFT_CS    10
#define TFT_RST   6 
#define TFT_DC    7 
#define TFT_SCLK  13   
#define TFT_MOSI  11   

// Visual Indicator LED
#define COMM_LED 9

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
#define DIRO_PIN 8
char address = '0';
String commandBuffer = "";
String dataBuffer = "+0.00+0.00+0.00+0.00+0.00";

// --- NON-BLOCKING TRANSMISSION VARIABLES ---
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
volatile float luxTotal = 0.0;
volatile unsigned long luxSampleCount = 0;

volatile float bmeTemp = 0.0;
volatile float bmePress = 0.0;
volatile float bmeHum = 0.0;
volatile float bmeGas = 0.0;

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

// --- WATCHDOG TIMER DEFINITIONS ---
#define WDT_KEY (0xA5)

// Overrides the default weak alias in the Arduino Due core that disables the WDT on boot
void watchdogSetup(void) {
  // Leave empty to prevent the core from disabling the watchdog
}

// --- FUNCTION PROTOTYPES ---
void handleCommand(String cmd);
void triggerSDI12Response(String message);
void handleSDI12TX();
void processIncomingUART();
void executeLogCycle();
String getTimestamp();
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

// HD Prototypes
void processCommandFSM(char c);

void setup() {
  Serial.begin(9600);
  while (!Serial); 
  Serial.println("--- Advanced Event-Driven Node Booting ---");
  Serial.println("Test4");

  // ==========================================================
  // AUTHOR: Jaymond Martin (102579706)
  // HD TASK: Watchdog Timer (WDT) Initialization
  // ==========================================================
  // Configure the WDT to trigger a full hardware reset if not fed.
  // Slow clock runs at 32.768 kHz. WDT frequency is 32768 / 128 = 256 Hz.
  // We set the timeout to 4 seconds (256 * 4) to safely accommodate the 3-second BME680 cycle.
  WDT->WDT_MR = WDT_MR_WDD(0xFFF) | 
                WDT_MR_WDRSTEN | 
                WDT_MR_WDV(256 * 4); 

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
    bme.setGasHeater(320, 150); 
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
  Timer3.attachInterrupt(luxSamplingISR);
  Timer3.start(150000); 

  Timer4.attachInterrupt(bmeSamplingISR);
  Timer4.start(3000000); 

  // 7. NVIC Priority Architecture
  NVIC_SetPriority(USART0_IRQn, 0); 
  NVIC_SetPriority(PIOA_IRQn, 1);
  for (int i = TC0_IRQn; i <= TC8_IRQn; i++) {
    NVIC_SetPriority((IRQn_Type)i, 4); 
  }

  Serial.println("System Ready. Fail-Safe WDT Active.");
  void printSDCardContents();
}

void loop() {
  // ==========================================================
  // AUTHOR: Jaymond Martin (102579706)
  // HD TASK: Watchdog Timer (WDT) Feeding
  // ==========================================================
  // Restart the WDT to prevent a hardware reset. If the loop freezes 
  // (e.g., stuck in an infinite I2C while-loop), the board will reboot.
  WDT->WDT_CR = WDT_CR_KEY(WDT_KEY) | WDT_CR_WDRSTT; // Comment this out to test wdt

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
    if (rxActivity || Serial1.available() > 0) {
      processIncomingUART();
      rxActivity = false; 
    }

    if (triggerLogCycle) {
      triggerLogCycle = false;
      executeLogCycle();
    }

    if (triggerClearSD) {
      triggerClearSD = false;
      clearSDCard();
    }

    if (commLedActive && (millis() - lastCommTime > 50)) {
      digitalWrite(COMM_LED, LOW);
      commLedActive = false;
    }

    if (millis() - lastDisplayTime >= displayInterval) {
      lastDisplayTime = millis();
      updateDataBuffer(); 
      updateDashboard(); 
    }

    if (millis() - lastLogTime >= logInterval) {
      lastLogTime = millis();
      executeLogCycle();
    }
    
    // TODO: Taha - Call your custom improvement logic here if necessary
  }
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Distinction - Dual Hardware Timer ISRs
// ==========================================================
void luxSamplingISR() {
  interrupts(); 
  float currentLux = lightMeter.readLightLevel();
  luxTotal += currentLux;
  luxSampleCount++;
}

void bmeSamplingISR() {
  interrupts(); 
  if (!bmeIsReading) {
    bmeEndTime = bme.beginReading();
    if (bmeEndTime != 0) {
      bmeIsReading = true;
    }
  }
}

void updateDataBuffer() {
  float safeTemp, safePress, safeHum, safeGas, safeLux;

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

  dataBuffer = "+" + String(safeTemp, 2) + 
               "+" + String(safePress, 2) + 
               "+" + String(safeHum, 2) + 
               "+" + String(safeGas, 2) + 
               "+" + String(safeLux, 2);
}

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// HD TASK: State-Machine Parser (FSM)
// ==========================================================
void processCommandFSM(char c) {
  // TODO: Chinmayee - Replace the old string-based handleCommand() logic with a 
  // robust Finite State Machine (FSM) that processes characters one by one.
  // This needs to handle partial or corrupted SDI-12 commands gracefully.
}

// ==========================================================
// LEGACY COMMAND HANDLER (To be replaced by FSM)
// ==========================================================
void handleCommand(String cmd) {
  cmd.trim();

  if (cmd == "?!") {
    triggerSDI12Response(String(address));
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'A' && cmd[3] == '!') {
    address = cmd[2]; 
    triggerSDI12Response(String(address));
  } else if (cmd == String(address) + "M!") {
    triggerSDI12Response(String(address) + "0035");
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'D' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      updateDataBuffer(); 
      triggerSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      triggerSDI12Response(String(address)); 
    }
  } else if (cmd.length() == 4 && cmd[0] == address && cmd[1] == 'R' && cmd[3] == '!') {
    if (cmd[2] == '0') {
      updateDataBuffer();
      triggerSDI12Response(String(address) + dataBuffer);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      triggerSDI12Response(String(address)); 
    }
  }
}

void triggerSDI12Response(String message) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Sending Response: "); 
  Serial.println(message);
  
  pendingTxMessage = message + "\r\n";
  txDuration = pendingTxMessage.length() * 10; 
  
  ignoreRX = true; 
  digitalWrite(DIRO_PIN, LOW); 
  
  txTimer = millis();
  txStage = 1; 
}

void handleSDI12TX() {
  if (txStage == 1) {
    if (millis() - txTimer >= 2) {
      Serial1.print(pendingTxMessage);
      txStage = 2;
    }
  } 
  else if (txStage == 2) {
    if (millis() - txTimer >= txDuration) {
      if (USART0->US_CSR & US_CSR_TXEMPTY) {
        digitalWrite(DIRO_PIN, HIGH); 
        txTimer = millis();
        txStage = 3;
      }
    }
  } 
  else if (txStage == 3) {
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
void commISR() {
  if (ignoreRX) return; 
  
  rxActivity = true; 
  digitalWrite(COMM_LED, HIGH);
  commLedActive = true;
  lastCommTime = millis(); 
}

void processIncomingUART() {
  while (Serial1.available() > 0) {
    char incomingByte = Serial1.read();

    if (incomingByte == 0 || incomingByte == 255) {
      commandBuffer = ""; 
      continue; 
    }

    // TODO: Chinmayee - Once processCommandFSM() is ready, pass 'incomingByte' 
    // directly to it here and remove the string buffering logic below.

    if (incomingByte != '\r' && incomingByte != '\n') {
      commandBuffer += incomingByte;
    }

    if (incomingByte == '!') {
      String cleanCmd = "";
      int len = commandBuffer.length();

      if (len >= 2 && commandBuffer.substring(len - 2) == "?!") {
        cleanCmd = "?!";
      } else if (len >= 3 && commandBuffer.endsWith("M!") && commandBuffer[len - 3] == address) {
        cleanCmd = commandBuffer.substring(len - 3);
      } else if (len >= 4 && commandBuffer.endsWith("!") && commandBuffer[len - 4] == address) {
        char cmdChar = commandBuffer[len - 3];
        if (cmdChar == 'A' || cmdChar == 'D' || cmdChar == 'R') {
          cleanCmd = commandBuffer.substring(len - 4);
        }
      }

      if (cleanCmd != "") {
        handleCommand(cleanCmd);
      }
      commandBuffer = ""; 
    }
    
    if (commandBuffer.length() > 30) {
      commandBuffer = "";
    }
  }
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Credit - Core Logging & Timestamp Generation
// ==========================================================
void executeLogCycle() {
  updateDataBuffer();

  //Serial.println("[TEST] Simulating a severe I2C lockup now. Board should freeze...");
  //delay(5000); // 5-second freeze. The WDT times out at 4 seconds!

  String timestamp = getTimestamp();
  String logEntry = timestamp + ", " + dataBuffer;

  Serial.print("[");
  Serial.print(millis());
  Serial.print("] Auto-Log Triggered: ");
  Serial.println(logEntry);

  logToSDCard(logEntry);
  updateDashboard();

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

void printSDCardContents() {
  Serial.println("\n--- Reading LOG.CSV ---");
  if (logFile.open("LOG.CSV", O_READ)) {
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

void btnLogISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastBtnLogTime > 200) { 
    triggerLogCycle = true;
    lastBtnLogTime = currentTime;
  }
}

void btnClearISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastBtnClearTime > 200) { 
    triggerClearSD = true;
    lastBtnClearTime = currentTime;
  }
}