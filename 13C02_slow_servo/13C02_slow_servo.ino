#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10  // servo motor

// duty duration for myservo.writeMicroseconds()
// calibrated for MG90S
#define _DUTY_MIN 550   // 0 degree
#define _DUTY_NEU 1475  // 90 degree
#define _DUTY_MAX 2400  // 180 degree

#define INTERVAL 20     // servo update interval (unit: msec)

// global variables
float _SERVO_SPEED = 3.0;         // servo speed limit (unit: degree/sec)
float duty_curr   = _DUTY_MIN;    // current duty (unit: usec)
float duty_target = _DUTY_MAX;    // target duty  (unit: usec)
int   experiment  = 1;            // 1: speed 3 (0->180 deg, 60 s), 2: speed 0.3 (180->90 deg, 300 s)
unsigned long last_sampling_time; // unit: msec

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_MIN);
  Serial.begin(57600);

  delay(2000);                    // wait until the servo reaches 0 degree
  last_sampling_time = millis();
}

void loop() {
  // wait until next sampling time
  if (millis() < last_sampling_time + INTERVAL)
    return;
  last_sampling_time += INTERVAL;

  // duty change allowed in one interval: full range(180 deg) * speed / 180 * interval(sec)
  float duty_chg = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  // move toward the target at constant speed (never overshoot)
  if (duty_curr < duty_target)
    duty_curr = min(duty_curr + duty_chg, duty_target);
  else
    duty_curr = max(duty_curr - duty_chg, duty_target);
  myservo.writeMicroseconds(duty_curr);

  // experiment 1 done -> start experiment 2 from where we are (180 deg, no jump)
  if (experiment == 1 && duty_curr == duty_target) {
    experiment = 2;
    _SERVO_SPEED = 0.3;
    duty_target = _DUTY_NEU;      // 180 -> 90 degree = 90 degree travel
  }

  // output to serial plotter
  Serial.print("Speed:");  Serial.print(_SERVO_SPEED);
  Serial.print(",Angle:"); Serial.print((duty_curr - _DUTY_MIN) * 180.0 / (_DUTY_MAX - _DUTY_MIN));
  Serial.println("");
}
