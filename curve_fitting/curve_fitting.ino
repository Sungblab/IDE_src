// 20C10: automatic curve fitting for the Sharp IR distance sensor
// Serial monitor: baud 1000000, line ending "Newline" or "Both NL & CR".
//
// Flow: enter "method degree" -> put the ball at 0, 5, ..., 30 cm and press Enter each time
//       -> the sketch prints a ready-to-paste volt_to_distance() function.
//
// method 0: distance = polynomial of a_value
// method 1: distance = polynomial of 1000 / a_value (the IR sensor is roughly hyperbolic)

#define PIN_IR   A0
#define N_POS    7      // ball positions: 0, 5, 10, ..., 30 cm
#define STEP_MM  50
#define N_READ   20     // readings per position; the median removes the sensor's spiky noise
#define MAX_DEG  5

unsigned int adc[N_POS];          // measured sensor values (0 ~ 1023)
float coef[MAX_DEG + 1];          // fitted coefficients, lowest order first
float mid, half;                  // normalisation: t = (u - mid) / half, so t is in -1 ~ 1
int method, degree;

int cmp(const void *a, const void *b) {
  return *(unsigned int *)a - *(unsigned int *)b;
}

unsigned int measure() {
  unsigned int v[N_READ];
  for (int i = 0; i < N_READ; i++) v[i] = analogRead(PIN_IR);
  qsort(v, N_READ, sizeof(v[0]), cmp);
  return v[N_READ / 2];
}

// Blocks until a line is typed; buf gets it without CR/LF (just pressing Enter gives "").
void read_line(char *buf, int len) {
  int i = 0;
  while (Serial.available() == 0)
    ;
  delay(50);  // ponytail: assumes the whole line arrives within 50 ms (true for the IDE monitor)
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\r' && c != '\n' && i < len - 1) buf[i++] = c;
  }
  buf[i] = 0;
}

float transform(float a) {
  return method ? 1000.0 / a : a;
}

// Float on AVR has only ~7 digits, so fit against the normalised t instead of raw a_value.
float eval(float a) {
  float t = (transform(a) - mid) / half, s = 0;
  for (int k = degree; k >= 0; k--) s = s * t + coef[k];
  return s;
}

// Least squares via the normal equations + Gaussian elimination. Returns false if it fails.
bool fit() {
  float u[N_POS], umin = 1e30, umax = -1e30;
  for (int i = 0; i < N_POS; i++) {
    if (adc[i] == 0) return false;
    u[i] = transform(adc[i]);
    if (u[i] < umin) umin = u[i];
    if (u[i] > umax) umax = u[i];
  }
  if (umax - umin < 1e-3) return false;
  mid = (umin + umax) / 2;
  half = (umax - umin) / 2;

  int n = degree + 1;
  float m[MAX_DEG + 1][MAX_DEG + 2] = {};     // augmented matrix [A | b]
  for (int i = 0; i < N_POS; i++) {
    float t = (u[i] - mid) / half, pw[2 * MAX_DEG + 1];
    pw[0] = 1;
    for (int k = 1; k <= 2 * degree; k++) pw[k] = pw[k - 1] * t;
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < n; k++) m[j][k] += pw[j + k];
      m[j][n] += i * STEP_MM * pw[j];
    }
  }

  for (int c = 0; c < n; c++) {
    int p = c;                                // partial pivoting
    for (int r = c + 1; r < n; r++) if (fabs(m[r][c]) > fabs(m[p][c])) p = r;
    for (int k = 0; k <= n; k++) { float s = m[c][k]; m[c][k] = m[p][k]; m[p][k] = s; }
    if (fabs(m[c][c]) < 1e-6) return false;
    for (int r = c + 1; r < n; r++) {
      float f = m[r][c] / m[c][c];
      for (int k = c; k <= n; k++) m[r][k] -= f * m[c][k];
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    float s = m[r][n];
    for (int k = r + 1; k < n; k++) s -= m[r][k] * coef[k];
    coef[r] = s / m[r][r];
  }
  return true;
}

void print_result() {
  float ss_res = 0, ss_tot = 0, max_err = 0, mean_y = (N_POS - 1) * STEP_MM / 2.0;
  for (int i = 0; i < N_POS; i++) {
    float y = i * STEP_MM, e = eval(adc[i]) - y;
    ss_res += e * e;
    ss_tot += (y - mean_y) * (y - mean_y);
    if (fabs(e) > max_err) max_err = fabs(e);
  }
  Serial.println();
  Serial.print(F("// method ")); Serial.print(method);
  Serial.print(F(", degree ")); Serial.print(degree);
  Serial.print(F(", R2 = ")); Serial.print(1 - ss_res / ss_tot, 5);
  Serial.print(F(", max error on the 7 points = ")); Serial.print(max_err, 1); Serial.println(F(" mm"));
  Serial.print(F("// valid for a_value in about ")); Serial.print(adc[N_POS - 1]);
  Serial.print(F(" ~ ")); Serial.println(adc[0]);
  Serial.println(F("float volt_to_distance(unsigned int a_value)"));
  Serial.println(F("{"));
  Serial.print(F("  float t = (")); Serial.print(method ? F("1000.0 / a_value") : F("a_value"));
  Serial.print(F(" - ")); Serial.print(mid, 6);
  Serial.print(F(") / ")); Serial.print(half, 6); Serial.println(F(";"));
  Serial.print(F("  return ")); Serial.print(coef[0], 6);
  for (int k = 1; k <= degree; k++) {
    Serial.print(coef[k] < 0 ? F(" - ") : F(" + ")); Serial.print(fabs(coef[k]), 6);
    for (int j = 0; j < k; j++) Serial.print(F(" * t"));
  }
  Serial.println(F(";"));
  Serial.println(F("}"));
  Serial.println();
}

void setup() {
  Serial.begin(1000000);
}

void loop() {
  char buf[16];
  while (Serial.available()) Serial.read();   // drop stale input

  do {
    Serial.println(F("Enter \"method degree\" (method 0 or 1, degree 1~5), e.g. 0 3"));
    read_line(buf, sizeof(buf));
  } while (sscanf(buf, "%d %d", &method, &degree) != 2
           || method < 0 || method > 1 || degree < 1 || degree > MAX_DEG);

  for (int i = 0; i < N_POS; i++) {
    Serial.print(F("Put the ball at ")); Serial.print(i * STEP_MM / 10);
    Serial.println(F(" cm and press Enter"));
    read_line(buf, sizeof(buf));
    adc[i] = measure();
    Serial.print(F("  sensor value: ")); Serial.println(adc[i]);
  }

  if (fit()) print_result();
  else Serial.println(F("Fit failed: a sensor value is 0 or all values are the same. Check the wiring."));
  Serial.println(F("--- start over ---"));
}
