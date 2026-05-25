/*
 * Unit: ENG20009 Engineering Technology Inquiry Project
 * Group 2 Members:
 * - Jaymond Martin (102579706)   [Task: Read from both sensors]
 * - Chinmayee Sharma (105702631) [Task: Command handling look at table]
 * - Taha Mohamed Shaik (105910382) [Task: Check Serial monitor constantly]
 * 
 * Description: Project Pass Task - Compliant Sensor Node
 * This program implements the core SDI-12 logic to allow the Arduino to act 
 * as a "Slave" sensor. It continuously polls the UART for commands, parses 
 * them according to the SDI-12 specification, reads environmental data from 
 * the BME680 and BH1750 sensors via I2C, and returns the concatenated values.
 * 
 * Hardware Mapping (Swinburne Arduino Due Board):
 * - BME680 (Temp, Humidity, Pressure, Gas) -> I2C (SDA/SCL) at 0x76
 * - BH1750 (Light Intensity) -> I2C (SDA/SCL) at 0x23
 * - SDI-12 UART Converter -> Serial1 (TX1=18, RX1=19), DIRO Control -> Pin 7
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <BH1750.h>

// --- SENSOR OBJECT INITIALIZATION ---
Adafruit_BME680 bme;
BH1750 lightMeter;

// --- SDI-12 STATE VARIABLES ---
#define DIRO_PIN 7 // Direction control pin for UART to SDI-12 converter
char address = '0';
String command = "";
String dataBuffer = "+0.00+0.00+0.00+0.00+0.00"; // Default empty buffer

// Function prototypes
void handleCommand(String cmd);
void sendSDI12Response(String message);
void readSensors();

void setup() {
  // Initialize standard Serial for IDE debugging
  Serial.begin(9600);
  Serial.println("--- SDI-12 Environmental Sensor Node Booting ---");

  // Initialize SDI-12 UART Communication via TX1/RX1
  // SDI-12 requires 1200 baud, 7 data bits, even parity, and 1 stop bit
  Serial1.begin(1200, SERIAL_7E1);
  pinMode(DIRO_PIN, OUTPUT);
  digitalWrite(DIRO_PIN, HIGH); // Set DIRO HIGH to Receive mode by default
  
  // Initialize I2C Bus
  Wire.begin();

  // Initialize BH1750 Light Sensor
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Error: BH1750 initialization failed! Check I2C wiring.");
  } else {
    Serial.println("BH1750 Light Sensor initialized.");
  }

  // Initialize BME680 Environmental Sensor
  if (!bme.begin(0x76)) {
    Serial.println("Error: BME680 initialization failed! Check I2C wiring.");
  } else {
    Serial.println("BME680 Environmental Sensor initialized.");
    // Configure BME680 oversampling and filter settings for accurate readings
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

  Serial.println("System Ready. Listening for SDI-12 commands on Serial1...");
}

void loop() {
  // ==========================================================
  // AUTHOR: Taha Mohamed Shaik (105910382)
  // TASK: Check Serial monitor constantly (UART Polling)
  // ==========================================================
  
  // Continuously monitor the UART port connected to the SDI-12 converter
  if (Serial1.available()) {
    int incomingByte = Serial1.read();

    // SDI-12 commands ALWAYS terminate with an exclamation mark '!' (ASCII 33)
    if (incomingByte == 33) {
      // Echo the received command to the IDE Serial Monitor for debugging
      Serial.print("Received Command: ");
      Serial.println(command + "!");
      
      // Pass the complete command string to the parser
      handleCommand(command + "!");
      
      // Wipe the buffer clean immediately so it's ready for the next command
      command = ""; 
    } else {
      // Ignore start bits (0) and append valid characters to the buffer
      if (incomingByte != 0 && incomingByte != '\r' && incomingByte != '\n') {
        command += char(incomingByte);
      }
    }
  }
}

// ==========================================================
// AUTHOR: Chinmayee Sharma (105702631)
// TASK: Command handling look at table (SDI-12 Parser)
// ==========================================================
void handleCommand(String cmd) {
  cmd.trim();

  // 1. Address Query: ?!
  if (cmd == "?!") {
    sendSDI12Response(String(address));
  }

  // 2. Change Address: aAb! (e.g., 0A1! changes address from 0 to 1)
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'A' &&
           cmd[3] == '!') {

    address = cmd[2]; // Extract the new address
    sendSDI12Response(String(address));
  }
  
  // 3. Start Measurement: aM!
  else if (cmd == String(address) + "M!") {
    readSensors();

    // Indicates: 003 seconds until 5 values are ready (Temp, Press, Hum, Gas, Lux)
    String response = String(address) + "0035"; 
    sendSDI12Response(response);
  }
  
  // 4. Send Data: aD0! to aD9!
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'D' &&
           cmd[3] == '!') {
           
    if (cmd[2] == '0') {
      // D0 requests the primary data payload
      String response = String(address) + dataBuffer;
      sendSDI12Response(response);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      // D1-D9 requests additional data, but the payload fits entirely in D0.
      // Returning just the address indicates there is no more data to send.
      sendSDI12Response(String(address)); 
    }
  }
  
  // 5. Continuous Measurement: aR0! to aR9!
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'R' &&
           cmd[3] == '!') {

    if (cmd[2] == '0') {
      // R0 requests a continuous reading
      readSensors();
      String response = String(address) + dataBuffer;
      sendSDI12Response(response);
    } else if (cmd[2] > '0' && cmd[2] <= '9') {
      // R1-R9 requests additional data, but the payload fits entirely in R0.
      sendSDI12Response(String(address)); 
    }
  }
}

// Helper function to handle the half-duplex DIRO pin toggling for SDI-12 transmission
void sendSDI12Response(String message) {
  Serial.print("Sending Response: "); 
  Serial.println(message);
  
  digitalWrite(DIRO_PIN, LOW); // Set DIRO LOW to Transmit
  delay(100); // Brief delay to allow the bus to stabilize
  
  Serial1.print(message + "\r\n");
  Serial1.flush(); // Wait for the transmission to completely finish
  
  // Reset the Serial1 receiver to clear any garbage data generated during transmission
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  
  digitalWrite(DIRO_PIN, HIGH); // Set DIRO HIGH to return to Receive mode
}

// ==========================================================
// AUTHOR: Jaymond Martin (102579706)
// TASK: Read from both sensors (Light & Temp)
// ==========================================================
void readSensors() {
  // Trigger a new reading from the BME680
  if (!bme.performReading()) {
    Serial.println("Failed to perform BME680 reading");
    return;
  }

  // Read light intensity from the BH1750
  float lux = lightMeter.readLightLevel();

  // Format the data into the standard SDI-12 concatenated string format.
  // The required format is: +Temperature+Pressure+Humidity+Gas+Lux
  // Example output: +29.17+1004.46+57.64+66.06+270.00
  
  dataBuffer = ""; // Clear the previous reading
  
  // Append Temperature (*C)
  dataBuffer += "+";
  dataBuffer += String(bme.temperature, 2);
  
  // Append Pressure (hPa)
  dataBuffer += "+";
  dataBuffer += String(bme.pressure / 100.0, 2);
  
  // Append Humidity (%)
  dataBuffer += "+";
  dataBuffer += String(bme.humidity, 2);
  
  // Append Gas Resistance (KOhms)
  dataBuffer += "+";
  dataBuffer += String(bme.gas_resistance / 1000.0, 2);
  
  // Append Light Intensity (Lux)
  dataBuffer += "+";
  dataBuffer += String(lux, 2);
  
  Serial.println("Sensors Read Successfully. Buffer Updated.");
}