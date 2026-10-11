// Sit-up counter — Seeed XIAO nRF52840 Sense
//
// Board package: "Seeed nRF52 Boards" (NOT the "mbed-enabled" one)
//   Board: Seeed XIAO nRF52840 Sense
// Library: "Seeed Arduino LSM6DS3" (Library Manager)
//
// How it works:
//   - Low-pass filters the accelerometer to get the gravity direction.
//   - On calibrate, stores the "lying flat" gravity direction as reference.
//   - Torso angle = angle between current gravity and the reference.
//     This works no matter which way the board is mounted on the strap.
//   - State machine: DOWN -> RISING -> UP -> back DOWN = 1 rep.
//     Rising past PARTIAL_DEG but dropping back before UP_DEG = partial rep.
//
// Reps are only counted during a set (between Start and Stop).
//
// Commands (USB serial or BLE UART):
//   g = start set (lie flat first: recalibrates, resets counts, starts counting)
//   x = stop set
//   c = calibrate only, r = reset counts, p = toggle plotter output, s = status
//
// BLE: advertises as "SitupCounter" using the Nordic UART service.
// Messages:
//   STARTED
//   REP,<count>,<ms since last rep>
//   PARTIAL,<count>
//   STOPPED,<reps>,<partials>,<set ms>
//   STATUS,<reps>,<partials>,<angle>,<armed 0/1>,<set ms so far>
//   CALIBRATED, RESET

#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>

// ---------- Tuning ----------
const float UP_DEG      = 55.0;  // torso angle that counts as "up"
const float DOWN_DEG    = 20.0;  // must return below this to finish a rep
const float START_DEG   = 25.0;  // leaving DOWN starts a rep attempt
const float PARTIAL_DEG = 35.0;  // reached this but not UP_DEG = partial
const float ALPHA       = 0.15;  // low-pass factor at 50 Hz (lower = smoother)
const uint32_t SAMPLE_MS  = 20;  // 50 Hz
const uint32_t MIN_REP_MS = 600; // ignore reps faster than this

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;

float gx = 0, gy = 0, gz = 1;     // filtered gravity vector (g)
float rx = 0, ry = 0, rz = 1;     // reference "lying flat" unit vector

enum State { ST_DOWN, ST_RISING, ST_UP };
State state = ST_DOWN;
float peakDeg = 0;
uint32_t reps = 0, partials = 0, lastRepMs = 0;
bool plotMode = true;
bool armed = false;               // true only during a set
uint32_t setStartMs = 0;

uint32_t lastSampleMs = 0, ledOffAt = 0;

// ---------- LEDs (active LOW) ----------
void ledsOff() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}
void flash(int pin, uint32_t ms) {
  ledsOff();
  digitalWrite(pin, LOW);
  ledOffAt = millis() + ms;
}

// ---------- Output ----------
void send(const String &msg) {
  if (!plotMode) Serial.println(msg);
  if (Bluefruit.connected()) bleuart.println(msg);
}

// ---------- Sensor ----------
void readFiltered() {
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  gx += ALPHA * (ax - gx);
  gy += ALPHA * (ay - gy);
  gz += ALPHA * (az - gz);
}

float angleDeg() {
  float n = sqrt(gx * gx + gy * gy + gz * gz);
  if (n < 0.1) return 0;
  float d = (gx * rx + gy * ry + gz * rz) / n;
  d = constrain(d, -1.0f, 1.0f);
  return degrees(acos(d));
}

void calibrate() {
  digitalWrite(LED_BLUE, LOW);
  float sx = 0, sy = 0, sz = 0;
  const int N = 50;                       // ~1 second
  for (int i = 0; i < N; i++) {
    readFiltered();
    sx += gx; sy += gy; sz += gz;
    delay(SAMPLE_MS);
  }
  float n = sqrt(sx * sx + sy * sy + sz * sz);
  if (n > 0.1) { rx = sx / n; ry = sy / n; rz = sz / n; }
  state = ST_DOWN;
  peakDeg = 0;
  ledsOff();
  send("CALIBRATED");
}

// ---------- Rep logic ----------
void onRep(uint32_t now) {
  if (lastRepMs && now - lastRepMs < MIN_REP_MS) return;
  uint32_t dt = lastRepMs ? now - lastRepMs : 0;
  lastRepMs = now;
  reps++;
  send("REP," + String(reps) + "," + String(dt));
  flash(LED_GREEN, 150);
}

void onPartial() {
  partials++;
  send("PARTIAL," + String(partials));
  flash(LED_RED, 300);
}

void updateReps(float a, uint32_t now) {
  switch (state) {
    case ST_DOWN:
      if (a > START_DEG) { state = ST_RISING; peakDeg = a; }
      break;
    case ST_RISING:
      if (a > peakDeg) peakDeg = a;
      if (a >= UP_DEG) {
        state = ST_UP;
      } else if (a < DOWN_DEG) {
        if (peakDeg >= PARTIAL_DEG) onPartial();
        state = ST_DOWN;
      }
      break;
    case ST_UP:
      if (a < DOWN_DEG) { onRep(now); state = ST_DOWN; }
      break;
  }
}

// ---------- Commands ----------
void handleCmd(char c) {
  switch (tolower(c)) {
    case 'c': calibrate(); break;
    case 'r':
      reps = partials = lastRepMs = 0;
      state = ST_DOWN;
      send("RESET");
      break;
    case 'g':                       // start a set
      reps = partials = lastRepMs = 0;
      calibrate();                  // you should be lying flat
      armed = true;
      setStartMs = millis();
      send("STARTED");
      flash(LED_GREEN, 400);
      break;
    case 'x':                       // stop the set
      send("STOPPED," + String(reps) + "," + String(partials) + "," +
           String(armed ? millis() - setStartMs : 0));
      armed = false;
      flash(LED_BLUE, 400);
      break;
    case 'p': plotMode = !plotMode; break;
    case 's':
      send("STATUS," + String(reps) + "," + String(partials) + "," +
           String(angleDeg(), 1) + "," + String(armed ? 1 : 0) + "," +
           String(armed ? millis() - setStartMs : 0));
      break;
  }
}

// ---------- BLE ----------
void startBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("SitupCounter");
  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledsOff();

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);   // IMU power rail
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);
#endif

  if (imu.begin() != 0) {
    // IMU not found: blink red forever and say why on Serial
    while (true) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      Serial.println("ERROR: IMU not found. Is Tools > Board set to "
                     "'Seeed XIAO nRF52840 Sense'?");
      delay(500);
    }
  }

  startBLE();

  // Seed the filter, then give 3 s to lie flat before calibrating
  gx = imu.readFloatAccelX();
  gy = imu.readFloatAccelY();
  gz = imu.readFloatAccelZ();
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BLUE, i % 2 ? HIGH : LOW);
    delay(500);
  }
  calibrate();
}

void loop() {
  uint32_t now = millis();

  while (Serial.available()) handleCmd(Serial.read());
  while (bleuart.available()) handleCmd(bleuart.read());

  if (ledOffAt && now >= ledOffAt) { ledsOff(); ledOffAt = 0; }

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = now;
    readFiltered();
    float a = angleDeg();
    if (armed) updateReps(a, now);

    if (plotMode) {
      Serial.print("angle:");  Serial.print(a, 1);
      Serial.print(",up:");    Serial.print(UP_DEG, 0);
      Serial.print(",down:");  Serial.print(DOWN_DEG, 0);
      Serial.print(",reps:");  Serial.println(reps);
    }
  }
}
