// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define N 10              // number of samples for median filter (3, 10, 30으로 변경하며 테스트)
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_ema = _DIST_MAX;         // EMA distance

float samples[N];                   // 최근 N개 샘플 저장 (원형 버퍼)
int sample_idx = 0;                 // 다음에 저장할 위치
int sample_count = 0;               // 현재까지 저장된 샘플 수 (최대 N)

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_median;
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  // median filter (범위 필터 / 직전 유효값 코드 제거)
  dist_median = median_filter(dist_raw);

  // EMA filter
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // output the read value to the serial port
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",raw:");    Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",median:"); Serial.print(dist_median);
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");

  // do something here
  if ((dist_median < _DIST_MIN) || (dist_median > _DIST_MAX))
    digitalWrite(PIN_LED, 1);       // LED OFF
  else
    digitalWrite(PIN_LED, 0);       // LED ON

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// 새 샘플을 버퍼에 넣고, 최근 N개 샘플의 중위수를 반환
float median_filter(float new_sample)
{
  // 1. 원형 버퍼에 저장 (가장 오래된 값을 덮어씀)
  samples[sample_idx] = new_sample;
  sample_idx = (sample_idx + 1) % N;
  if (sample_count < N) sample_count++;

  // 2. 정렬용 배열에 복사 (원본 버퍼 순서는 유지)
  float sorted[N];
  for (int i = 0; i < sample_count; i++)
    sorted[i] = samples[i];

  // 3. 삽입 정렬
  for (int i = 1; i < sample_count; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  // 4. 중위수 반환 (짝수 개면 가운데 두 값의 평균)
  if (sample_count % 2 == 1)
    return sorted[sample_count / 2];
  else
    return (sorted[sample_count / 2 - 1] + sorted[sample_count / 2]) / 2.0;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
