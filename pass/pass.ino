// Libraries

// Pin Definitions

// Variables
// const char myAddress = '0'; // Default factory address for SDI-12 sensors
// const String commandBuffer = "";  // Buffer to hold incoming UART characters

// Main Functions
void setup() {
  Serial.begin(9600);
  
  Serial.println("--- SDI-12 Environmental Sensor Node Booting ---");
  Serial.println("Type '?!', '0I!', '0M!', or '0D0!' to test.");
}

void loop() {
}