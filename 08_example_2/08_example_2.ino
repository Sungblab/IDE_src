// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)  <-- 25ms 적용
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)
#define _DIST_PEAK 200.0  // distance with maximum brightness (unit: mm)

#define TIMEOUT ((INTERVAL / 2.0) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)       // coefficent to convert duration to distance

unsigned long last_sampling_time = 0;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar
  analogWrite(PIN_LED, 255);    // LED OFF (active low)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance;
  int led_pwm;   // analogWrite 값: 0 = 가장 밝음, 255 = 꺼짐 (active low)

  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((distance == 0.0) || (distance > _DIST_MAX)) {
    distance = _DIST_MAX + 10.0;    // Set Higher Value
    led_pwm = 255;                  // LED OFF
  } else if (distance < _DIST_MIN) {
    distance = _DIST_MIN - 10.0;    // Set Lower Value
    led_pwm = 255;                  // LED OFF
  } else {    // In desired Range: 100mm ~ 300mm
    led_pwm = distance_to_pwm(distance);
  }
  analogWrite(PIN_LED, led_pwm);

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",pwm:");       Serial.print(led_pwm);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");

  // delay(50) 삭제

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// 거리(mm) -> analogWrite 값 (active low)
//  200mm: 0 (최대 밝기), 150/250mm: 약 127 (duty 50%), 100/300mm: 255 (최소)
int distance_to_pwm(float dist)
{
  float offset = fabs(dist - _DIST_PEAK);                  // 0 ~ 100 (mm)
  float ratio  = offset / (_DIST_MAX - _DIST_PEAK);        // 0.0 ~ 1.0 (float 연산!)
  if (ratio > 1.0) ratio = 1.0;
  return (int)(ratio * 255.0 + 0.5);                       // 반올림
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
