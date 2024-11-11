#include <ModbusMaster.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1); // Using Serial1 (TX=17, RX=16)
ModbusMaster node;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize RS485 Serial with baud rate 4800 as per SEM1000 datasheet
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Initialize Modbus with Slave ID 1 (default for SEM1000)
  node.begin(8, MySerial); // Slave ID 1 according to SEM1000 settings

  Serial.println("Modbus RTU Test for SEM1000 Weather Station");

  // Wind speed calibration
  uint8_t result = node.writeSingleRegister(6001, 0xAA);
  if (result == node.ku8MBSuccess) {
    Serial.println("Wind speed zero calibration successful");
  } else {
    Serial.print("Wind speed zero calibration failed, error code: ");
    Serial.println(result);
  }

  delay(10000); // Wait 10 seconds for calibration to complete
}

void loop() {
  uint8_t result;

  // Reading 10 registers from address 0x01F4 (500 in decimal) for wind speed, direction, temperature, humidity, and pressure
  result = node.readHoldingRegisters(500, 10);

  if (result == node.ku8MBSuccess) {
    // Reading values from registers and converting as per SEM1000 datasheet
    float windSpeed = node.getResponseBuffer(0) / 100.0;  // Wind speed in m/s (register 500, scaled by 100)
    float windDirection = node.getResponseBuffer(3);       // Wind direction in degrees (register 503)
    float temperature = node.getResponseBuffer(5) / 10.0;  // Temperature in °C (register 505, scaled by 10)
    float humidity = node.getResponseBuffer(4) / 10.0;     // Humidity in % (register 504, scaled by 10)
    float airPressure = node.getResponseBuffer(9) / 10.0;  // Air pressure in kPa (register 509, scaled by 10)

    // Displaying read values
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    Serial.print("Wind Direction: ");
    Serial.print(windDirection);
    Serial.println(" °");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Air Pressure: ");
    Serial.print(airPressure);
    Serial.println(" kPa");

  } else {
    Serial.print("Failed to read data, error code: ");
    Serial.println(result);
  }

  delay(2000);  // Wait 2 seconds before the next reading
}
