// Holds the servo at neutral (1500 us) so the horn can be mounted with the rail level.
// No potentiometer involved: the position cannot drift with the knob.
#include <Servo.h>

#define PIN_SERVO 10
#define _DUTY_NEU 1400

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
}

void loop() {
}
