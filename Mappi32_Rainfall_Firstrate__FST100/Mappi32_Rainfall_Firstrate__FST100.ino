// Rainfall Firstrate FST100

#include <HardwareSerial.h>

#define RXD2 16  // Pin RX ESP32
#define TXD2 17  // Pin TX ESP32

HardwareSerial mySerial(2); // Menggunakan UART2 di ESP32

// Variabel untuk menyimpan total curah hujan
float totalRainfall = 0.0;
uint16_t lastTicks = 0; // Variabel untuk menyimpan ketukan terakhir

void setup() {
  Serial.begin(115200);       // Serial untuk debugging
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Serial untuk RS485
  delay(100);
  Serial.println("Starting communication with rainfall sensor...");
}

void loop() {
  // Kirim perintah untuk membaca data sensor (menyesuaikan dengan protokol MODBUS-RTU sensor)
  uint8_t readCommand[] = {0x02, 0x03, 0x00, 0x2A, 0x00, 0x01, 0xA5, 0xF1};
  mySerial.write(readCommand, sizeof(readCommand));
  
  delay(100);  // Tunggu respons dari sensor

  // Cek apakah ada data yang diterima
  if (mySerial.available()) {
    Serial.println("Data received:");
    
    // Array untuk menyimpan data yang diterima
    uint8_t response[7];  // Menyesuaikan jumlah byte yang diterima dari sensor
    int index = 0;

    while (mySerial.available() && index < sizeof(response)) {
      response[index++] = mySerial.read();
    }
    
    // Proses data yang diterima
    if (index >= 5) {  // Cek apakah data yang diterima cukup
      uint16_t currentTicks = (response[3] << 8) | response[4];  // Menggabungkan byte menjadi angka 16-bit

      // Hanya tambahkan 0.2 mm per ketukan baru yang terdeteksi
      if (currentTicks > lastTicks) { // Ketukan baru terdeteksi
        totalRainfall += 0.2; // Tambahkan 0.2 mm
        lastTicks = currentTicks; // Update nilai ketukan terakhir
      }

      Serial.print("Total Rainfall (mm): ");
      Serial.println(totalRainfall);  // Tampilkan total curah hujan
    } else {
      Serial.println("Insufficient data received.");
    }
  } else {
    Serial.println("No data received.");
  }

  delay(1000);  // Jeda 1 detik sebelum membaca ulang
}
