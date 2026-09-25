#define PIR_PIN 27

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
}

void loop() {
  int pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH) {
    Serial.println("MOTION DETECTED");
  } else {
    Serial.println("NO MOTION");
  }

  delay(500);
}