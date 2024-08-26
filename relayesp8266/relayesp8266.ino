const int relayPin = D1;  // Pin GPIO D1 pada NodeMCU sebagai output untuk relay
const int buzzerPin = D2;  // Pin GPIO D2 pada NodeMCU sebagai output untuk buzzer

void setup() {
  pinMode(relayPin, OUTPUT);  // Set pin relay sebagai output
  pinMode(buzzerPin, OUTPUT);  // Set pin buzzer sebagai output
  
  digitalWrite(relayPin, LOW);  // Matikan relay secara awal
  digitalWrite(buzzerPin, LOW);  // Matikan buzzer secara awal
}

void loop() {
  // Baca status relay dari input pengguna (misalnya tombol atau sensor)
  bool relayStatus = digitalRead(relayPin);
  
  // Jika relay diaktifkan (HIGH), hidupkan buzzer
  if (relayStatus == HIGH) {
    digitalWrite(buzzerPin, HIGH);  // Hidupkan buzzer
  } else {
    digitalWrite(buzzerPin, LOW);  // Matikan buzzer
  }
}
