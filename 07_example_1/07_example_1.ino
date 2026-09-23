#define TRIG_PIN 12
#define ECHO_PIN 13

void setup() {
  Serial.begin(57600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("TIMEOUT");
  } else {
    float distance_cm = duration * 0.0343 / 2.0;

    Serial.print("duration = ");
    Serial.print(duration);
    Serial.print(" us, distance = ");
    Serial.print(distance_cm);
    Serial.println(" cm");
  }

  delay(500);
}
