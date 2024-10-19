//BGT-W87X (M1) Sensor 5 in 1



#include <ModbusMaster.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1); // Gunakan Serial1 (TX=17, RX=16)

ModbusMaster node;

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);

  // Inisialisasi RS485 Serial dengan baud rate yang sesuai
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Inisialisasi Modbus dengan ID Slave 8 (alamat sensor yang sudah diubah)
  node.begin(8, MySerial); // ID Slave 8, sesuaikan dengan pengaturan sensor

  Serial.println("Modbus RTU Test for BGT-W87X(M1) 5 in 1");
}

void loop() {
  uint8_t result;

  // Membaca 12 register dari alamat 0x0000 untuk Suhu dan Kelembaban
  result = node.readHoldingRegisters(0x0000, 13); // Baca suhu, kelembaban, tekanan, kecepatan, arah

  if (result == node.ku8MBSuccess) {
    // Membaca nilai dari register dan mengkonversi sesuai datasheet
    float temperature = node.getResponseBuffer(0) / 10.0;  // Suhu dalam °C
    float humidity = node.getResponseBuffer(1) / 10.0;     // Kelembaban dalam %
    float airPressure = node.getResponseBuffer(6) / 10.0;  // Tekanan udara dalam hPa
    float windSpeed = node.getResponseBuffer(11) / 100.0;   // Kecepatan angin dalam m/s
    uint16_t windDirection = node.getResponseBuffer(12);    // Arah angin dalam °

    // Menampilkan nilai yang dibaca
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Air Pressure: ");
    Serial.print(airPressure);
    Serial.println(" hPa");

    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    Serial.print("Wind Direction: ");
    Serial.print(windDirection);
    Serial.println(" °");
  } else {
    Serial.print("Gagal membaca data, kode error: ");
    Serial.println(result);
  }

  delay(2000);  // Tunggu 2 detik sebelum membaca lagi
}
