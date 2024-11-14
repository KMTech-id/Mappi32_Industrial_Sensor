#include <HardwareSerial.h>
#include <ModbusMaster.h>

#define RXD2 16  // Pin RX ESP32
#define TXD2 17  // Pin TX ESP32

// Membuat objek Serial dan Modbus
HardwareSerial mySerial(2); // UART2 untuk sensor Rainfall
ModbusMaster node;           // Modbus untuk sensor 5-in-1 dan Soil Sensor

// Variabel Rainfall
float totalRainfall = 0.0;
uint16_t lastTicks = 0;

// Variabel 5-in-1 Weather Sensor
float windSpeed, windDirection, temperature5in1, humidity, airPressure;

// Variabel Soil Sensor
float temperatureSoil, soilMoisture, electricalConductivity;

void setup() {
  Serial.begin(115200);       // Serial untuk debugging
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Serial untuk Rainfall RS485

  // Inisialisasi Serial RS485 untuk sensor 5-in-1 dan Soil Sensor
  node.begin(1, mySerial); // ID awal untuk Soil Sensor, akan diubah dinamis

  Serial.println("Initializing sensors...");
}

void loop() {
  // Membaca data dari sensor Rainfall
  readRainfallSensor();

  // Membaca data dari sensor 5-in-1
  read5in1Sensor();

  // Membaca data dari Soil Sensor
  readSoilSensor();

  delay(2000); // Jeda untuk setiap siklus pembacaan sensor
}

// Fungsi membaca sensor Rainfall
void readRainfallSensor() {
  uint8_t readCommand[] = {0x02, 0x03, 0x00, 0x2A, 0x00, 0x01, 0xA5, 0xF1};
  mySerial.write(readCommand, sizeof(readCommand));
  delay(100);

  if (mySerial.available()) {
    uint8_t response[7];
    int index = 0;

    while (mySerial.available() && index < sizeof(response)) {
      response[index++] = mySerial.read();
    }

    if (index >= 5) {
      uint16_t currentTicks = (response[3] << 8) | response[4];

      if (currentTicks > lastTicks) {
        totalRainfall += 0.2;
        lastTicks = currentTicks;
      }
      Serial.print("Total Rainfall (mm): ");
      Serial.println(totalRainfall);
    } else {
      Serial.println("Insufficient Rainfall data.");
    }
  } else {
    Serial.println("No Rainfall data received.");
  }
}

// Fungsi membaca sensor 5-in-1
void read5in1Sensor() {
  node.begin(8, mySerial); // Mengubah ke ID sensor 5-in-1
  uint8_t result = node.readHoldingRegisters(500, 10);

  if (result == node.ku8MBSuccess) {
    windSpeed = node.getResponseBuffer(0) / 100.0;
    windDirection = node.getResponseBuffer(3);
    temperature5in1 = node.getResponseBuffer(5) / 10.0;
    humidity = node.getResponseBuffer(4) / 10.0;
    airPressure = node.getResponseBuffer(9) / 10.0;

    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    Serial.print("Wind Direction: ");
    Serial.print(windDirection);
    Serial.println(" °");

    Serial.print("Temperature (5-in-1): ");
    Serial.print(temperature5in1);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Air Pressure: ");
    Serial.print(airPressure);
    Serial.println(" kPa");
  } else {
    Serial.print("Failed to read 5-in-1 data, error code: ");
    Serial.println(result);
  }
}

// Fungsi membaca sensor Soil
void readSoilSensor() {
  node.begin(1, mySerial); // Mengubah kembali ke ID sensor Soil
  uint8_t result = node.readHoldingRegisters(0x0000, 3);

  if (result == node.ku8MBSuccess) {
    temperatureSoil = node.getResponseBuffer(0) / 100.0;
    soilMoisture = node.getResponseBuffer(1) / 100.0;
    electricalConductivity = node.getResponseBuffer(2);

    Serial.print("Temperature (Soil): ");
    Serial.print(temperatureSoil);
    Serial.println(" °C");

    Serial.print("Soil Moisture (VWC): ");
    Serial.print(soilMoisture);
    Serial.println(" %");

    Serial.print("Electrical Conductivity (EC): ");
    Serial.print(electricalConductivity);
    Serial.println(" uS/cm");
  } else {
    Serial.print("Failed to read Soil data, error code: ");
    Serial.println(result);
  }
}
