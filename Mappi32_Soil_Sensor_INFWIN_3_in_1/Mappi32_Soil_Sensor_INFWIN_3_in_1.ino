//Soil Sensor INFWIN 3 in 1
#include <ModbusMaster.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1); // Gunakan Serial1 (TX=17, RX=16)

ModbusMaster node;

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);

  // Inisialisasi RS485 Serial dengan baud rate yang sesuai
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Inisialisasi Modbus
  node.begin(1, MySerial); // ID Slave 1, sesuaikan dengan pengaturan sensor

  Serial.println("Modbus RTU Test");
}

void loop() {
  uint8_t result;
  uint16_t data[10];

  // Baca data dari sensor (contoh alamat register dari datasheet)
  result = node.readHoldingRegisters(0x0000, 3); // Membaca temperature, VWC, EC

  if (result == node.ku8MBSuccess) {
    // Membaca nilai dari register dan mengkonversi sesuai datasheet
    int16_t temperature = node.getResponseBuffer(0);
    uint16_t vwc = node.getResponseBuffer(1);
    uint16_t ec = node.getResponseBuffer(2);

    // Menampilkan nilai yang dibaca
    Serial.print("Temperature: ");
    Serial.print(temperature / 100.0);
    Serial.println(" °C");
    Serial.print("Soil Moisture (VWC): ");
    Serial.print(vwc / 100.0);
    Serial.println(" %");
    Serial.print("Electrical Conductivity (EC): ");
    Serial.print(ec);
    Serial.println(" uS/cm");
  } else {
    Serial.print("Gagal membaca data, kode error: ");
    Serial.println(result);
  }

  delay(2000);
}
