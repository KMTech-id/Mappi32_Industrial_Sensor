//bgt v2
// Rainfal BGT 

// Pin definition
const int rainSensorPin = 23;  // Digital pin connected to the rain sensor

// Variables
volatile int rainPulseCount = 0;  // Counter for the sensor pulses
float rainfallAmount = 0.0;  // Total rainfall in mm
unsigned long lastRainTime = 0;  // Time of the last detected pulse
const unsigned long rainTimeout = 60000;  // Timeout in milliseconds (1 minute)

// Constant
const float mmPerPulse = 0.2;  // 0.2mm per pulse

// Interrupt service routine (ISR)
void IRAM_ATTR countRainPulse() {
  rainPulseCount++;
  lastRainTime = millis();  // Update the time when the last pulse was detected
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set the sensor pin as input
  pinMode(rainSensorPin, INPUT_PULLUP);

  // Attach an interrupt to the sensor pin (falling edge, i.e., when the bucket tips)
  attachInterrupt(digitalPinToInterrupt(rainSensorPin), countRainPulse, FALLING);

  Serial.println("Rainfall sensor is ready.");
}

void loop() {
  // Calculate the rainfall based on the pulse count
  rainfallAmount = rainPulseCount * mmPerPulse;

  // Print the current rainfall amount every 10 seconds
  Serial.print("Total rainfall: ");
  Serial.print(rainfallAmount);
  Serial.println(" mm");

  // Check if no rain has been detected for a while (1 minute)
  if (millis() - lastRainTime > rainTimeout && rainPulseCount > 0) {
    Serial.println("No rain detected for 1 minute. Resetting rainfall data...");
    rainPulseCount = 0;  // Reset the pulse count
    rainfallAmount = 0.0;  // Reset the rainfall amount
  }

  // Wait for 10 seconds before the next reading
  delay(10000);
}
