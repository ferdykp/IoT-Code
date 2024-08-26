const int flowSensorPin = 27; // Ganti dengan pin yang sesuai pada ESP32

volatile int flowPulseCount = 0;
float flowRate = 0;

void setup() {
  Serial.begin(9600);
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
}

void loop() {
  flowRate = (flowPulseCount * 2.25); // Konversi pulsa menjadi laju aliran (disesuaikan dengan sensor)
  
  Serial.print("Flow Rate: ");
  Serial.print(flowRate);
  Serial.println(" L/min");

  delay(1000); // Berhenti sejenak
}

void pulseCounter() {
  flowPulseCount++;
}
