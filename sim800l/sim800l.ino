#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial sim800lSerial(D3, D4);  // RX, TX for SIM800L
SoftwareSerial neo6mSerial(D6, D7);    // RX, TX for NEO-6M

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  sim800lSerial.begin(9600);
  neo6mSerial.begin(9600);
  
  // Inisialisasi SIM800L
  delay(1000);
  sim800lSerial.println("AT+CMGF=1"); // Set mode SMS
  delay(1000);
}

void loop() {
  // Baca data GPS dari NEO-6M
  while (neo6mSerial.available() > 0) {
    if (gps.encode(neo6mSerial.read())) {
      if (gps.location.isUpdated()) {
        // Mendapatkan data koordinat
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        
        // Membuat pesan SMS dengan data koordinat
        String message = "Latitude: " + String(latitude, 6) + ", Longitude: " + String(longitude, 6);

        // Kirim SMS
        sendSMS("+6282257932326", message);

        delay(5000); // Tunggu 5 detik sebelum mengirim data GPS lagi
      }
    }
  }
}

void sendSMS(String phoneNumber, String message) {
  sim800lSerial.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(1000);
  sim800lSerial.print(message);
  delay(100);
  sim800lSerial.write(0x1A); // Mengirim karakter Ctrl+Z untuk mengirim SMS
  delay(1000);
}
