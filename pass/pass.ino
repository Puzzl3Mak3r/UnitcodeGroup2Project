#include <Adafruit_BME680.h>
#include <BH1750.h>

char address = '0';
String command = "";
String dataBuffer = "+24.50+48.20+315.00";

void setup() {
  Serial.begin(9600);

  Serial.println("--- SDI-12 Environmental Sensor Node Booting ---");
  Serial.println("Type '?!', '0M!', '0D0!', '0R0!' or '0A1!' to test.");
}

void loop() {
  // ==========================================================
  // POLLING LOGIC: Continuously monitor UART
  // ==========================================================
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Ignore stray carriage returns and line feeds
    if (c == '\r' || c == '\n') continue;

    // Append the incoming character to our global buffer
    command += c;

    // SDI-12 commands ALWAYS terminate with an exclamation mark '!'
    if (c == '!') {
      handleCommand(command);
      
      // Wipe the buffer clean immediately so it's ready for the next command
      command = ""; 
    }
  }
}

// ==========================================================
// SDI-12 COMMAND PARSER
// ==========================================================
void handleCommand(String cmd) {
  cmd.trim();

  // 1. Address Query: ?!
  if (cmd == "?!") {
    Serial.print(address);
    Serial.print("\r\n");
  }

  // 2. Change Address: aAb!
  // 0A1! changes from 0 to 1
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'A' &&
           cmd[3] == '!') {

    address = cmd[2]; // Extract the new address

    Serial.print(address);
    Serial.print("\r\n");
  }
  
  // 3. Start Measurement: aM!
  else if (cmd == String(address) + "M!") {
    readSensors();

    Serial.print(address);
    Serial.print("0033"); // Indicates: 003 seconds until 3 values are ready
    Serial.print("\r\n");
  }
  
  // 4. Send Data: aD0! to aD9!
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'D' &&
           cmd[2] >= '0' &&
           cmd[2] <= '9' &&
           cmd[3] == '!') {
           
    Serial.print(address);
    Serial.print(dataBuffer);
    Serial.print("\r\n");
  }
  
  // 5. Continuous Measurement: aR0! to aR9!
  else if (cmd.length() == 4 &&
           cmd[0] == address &&
           cmd[1] == 'R' &&
           cmd[2] >= '0' &&
           cmd[2] <= '9' &&
           cmd[3] == '!') {

    readSensors();

    // FIXED: Replaced undefined sendResponse() with standard Serial.print
    // FIXED: Corrected 'databuffer' to 'dataBuffer'
    Serial.print(address);
    Serial.print(dataBuffer);
    Serial.print("\r\n");
  }
}

// ==========================================================
// HARDWARE FUNCTIONS
// ==========================================================
void readSensors() {
  // Dummy function so the code compiles. 
  // You will drop your BME680 and BH1750 reading logic here later!
  // e.g., dataBuffer = "+" + String(bme.temperature) + "+" + String(bme.humidity) ...
}