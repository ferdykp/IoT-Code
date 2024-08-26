 const int relayPin = D1;     // Pin GPIO D1 pada NodeMCU sebagai output untuk relay
const int buzzerPin = D2;    // Pin GPIO D2 pada NodeMCU sebagai output untuk buzzer
const int triggerPin = D3;   // Pin GPIO D3 pada NodeMCU sebagai output untuk trigger sensor ultrasonik
const int echoPin = D4;      // Pin GPIO D4 pada NodeMCU sebagai input untuk echo sensor ultrasonik

const int distanceThreshold = 10;   // Ambang batas jarak (tersesuaikan dengan kebutuhan)

void setup() {
  pinMode(relayPin, OUTPUT);        // Set pin relay sebagai output
  pinMode(buzzerPin, OUTPUT);       // Set pin buzzer sebagai output
  pinMode(triggerPin, OUTPUT);      // Set pin trigger sensor ultrasonik sebagai output
  pinMode(echoPin, INPUT);          // Set pin echo sensor ultrasonik sebagai input

  digitalWrite(relayPin, LOW);      // Matikan relay secara awal
  digitalWrite(buzzerPin, LOW);     // Matikan buzzer secara awal

  Serial.begin(9600);               // Inisialisasi Serial Monitor
}

void loop() {
  // Mengirim sinyal trigger ke sensor ultrasonik
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Membaca waktu pantulan echo
  long duration = pulseIn(echoPin, HIGH);

  // Menghitung jarak berdasarkan waktu pantulan
  float distance = duration * 0.034 / 2;

  // Menampilkan jarak pada Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Mengendalikan relay dan buzzer berdasarkan jarak
  if (distance <= distanceThreshold) {
    digitalWrite(relayPin, HIGH);   // Aktifkan relay
    digitalWrite(buzzerPin, HIGH);  // Aktifkan buzzer
  } else {
    digitalWrite(relayPin, LOW);    // Matikan relay
    digitalWrite(buzzerPin, LOW);   // Matikan buzzer
  }

  delay(1000);
}
