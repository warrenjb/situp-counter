// Sit-up counter — Seeed XIAO nRF52840 Sense
//
// Board package: "Seeed nRF52 Boards" (NOT the "mbed-enabled" one)
//   Board: Seeed XIAO nRF52840 Sense
// Library: "Seeed Arduino LSM6DS3" (Library Manager)
//
// How it works:
//   - Low-pass filters the accelerometer to get the gravity direction.
//   - Calibrate stores the "lying flat" gravity direction as zero.
//   - Torso angle = angle between current gravity and that reference,
//     so it works no matter which way the board is mounted on the strap.
//   - State machine: DOWN -> RISING -> UP -> back DOWN = 1 rep.
//     Rising past the partial threshold but not reaching "up" = partial rep.
//   - Reps only count during a set (between Start and Stop).
//   - Thresholds are adjustable over BLE and saved in flash.
//
// Commands (USB serial or BLE UART):
//   g = start set (lie flat first: recalibrates, resets counts, starts counting)
//   x = stop set
//   c = calibrate only     r = reset counts     s = status + thresholds
//   a = stream live angle (10/s)   z = stop angle stream
//   p = toggle Serial Plotter output
//   T,<up>,<down>,<partial>  then newline = set and save thresholds (degrees)
//
// Messages out:
//   STARTED | REP,<n>,<ms since last> | PARTIAL,<n> | STOPPED,<reps>,<partials>,<ms>
//   STATUS,<reps>,<partials>,<angle>,<armed 0/1>,<set ms so far>
//   CFG,<up>,<down>,<partial> | ANG,<angle> | CALIBRATED | RESET | SAVED | ERR,<why>

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <LSM6DS3.h>
#include <Wire.h>

using namespace Adafruit_LittleFS_Namespace;

// ---------- Thresholds (saved in flash) ----------
struct Config {
  uint32_t magic;
  float upDeg;       // torso angle that counts as "up"
  float downDeg;     // must return below this to finish a rep
  float partialDeg;  // reached this but not "up" = partial rep
};
const uint32_t CFG_MAGIC = 0x53555031;   // version tag
Config cfg = { CFG_MAGIC, 55.0, 20.0, 35.0 };
const char *CFG_PATH = "/situp_cfg.bin";
const float START_GAP = 5.0;             // rep attempt starts at downDeg + this

// ---------- Fixed tuning ----------
const float ALPHA = 0.15;                // low-pass factor at 50 Hz
const uint32_t SAMPLE_MS  = 20;          // 50 Hz
const uint32_t MIN_REP_MS = 600;         // ignore reps faster than this

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;

float gx = 0, gy = 0, gz = 1;            // filtered gravity vector (g)
float rx = 0, ry = 0, rz = 1;            // reference "lying flat" unit vector

enum State { ST_DOWN, ST_RISING, ST_UP };
State state = ST_DOWN;
float peakDeg = 0;
uint32_t reps = 0, partials = 0, lastRepMs = 0;
bool plotMode = true;
bool armed = false;                      // true only during a set
uint32_t setStartMs = 0;
bool streamAngle = false;
uint8_t streamDiv = 0;

uint32_t lastSampleMs = 0, ledOffAt = 0;

bool capturing = false;                  // reading a "T,..." line
String lineBuf;

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

void sendCfg() {
  send("CFG," + String(cfg.upDeg, 0) + "," + String(cfg.downDeg, 0) + "," +
       String(cfg.partialDeg, 0));
}

// ---------- Config storage ----------
bool validCfg(float up, float down, float partial) {
  return down >= 5 && partial >= down + 5 && up >= partial + 5 && up <= 150;
}

void loadConfig() {
  File f(InternalFS);
  if (f.open(CFG_PATH, FILE_O_READ)) {
    Config tmp;
    if (f.read(&tmp, sizeof(tmp)) == (int)sizeof(tmp) && tmp.magic == CFG_MAGIC &&
        validCfg(tmp.upDeg, tmp.downDeg, tmp.partialDeg)) {
      cfg = tmp;
    }
    f.close();
  }
}

bool saveConfig() {
  InternalFS.remove(CFG_PATH);
  File f(InternalFS);
  if (!f.open(CFG_PATH, FILE_O_WRITE)) return false;
  f.write((const uint8_t *)&cfg, sizeof(cfg));
  f.close();
  return true;
}

void parseThresholds(const String &s) {   // s looks like ",60,20,40"
  int a = s.indexOf(',');
  int b = s.indexOf(',', a + 1);
  int c = s.indexOf(',', b + 1);
  if (a < 0 || b < 0 || c < 0) { send("ERR,format"); return; }
  float up = s.substring(a + 1, b).toFloat();
  float down = s.substring(b + 1, c).toFloat();
  float partial = s.substring(c + 1).toFloat();
  if (!validCfg(up, down, partial)) { send("ERR,range"); return; }
  cfg.upDeg = up;
  cfg.downDeg = down;
  cfg.partialDeg = partial;
  state = ST_DOWN;
  send(saveConfig() ? "SAVED" : "ERR,save");
  sendCfg();
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
      if (a > cfg.downDeg + START_GAP) { state = ST_RISING; peakDeg = a; }
      break;
    case ST_RISING:
      if (a > peakDeg) peakDeg = a;
      if (a >= cfg.upDeg) {
        state = ST_UP;
      } else if (a < cfg.downDeg) {
        if (peakDeg >= cfg.partialDeg) onPartial();
        state = ST_DOWN;
      }
      break;
    case ST_UP:
      if (a < cfg.downDeg) { onRep(now); state = ST_DOWN; }
      break;
  }
}

// ---------- Commands ----------
void handleCmd(char c) {
  switch (tolower(c)) {
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
    case 'c': calibrate(); break;
    case 'r':
      reps = partials = lastRepMs = 0;
      state = ST_DOWN;
      send("RESET");
      break;
    case 'a': streamAngle = true; break;
    case 'z': streamAngle = false; break;
    case 'p': plotMode = !plotMode; break;
    case 's':
      send("STATUS," + String(reps) + "," + String(partials) + "," +
           String(angleDeg(), 1) + "," + String(armed ? 1 : 0) + "," +
           String(armed ? millis() - setStartMs : 0));
      sendCfg();
      break;
  }
}

void handleChar(char c) {
  if (capturing) {
    if (c == '\n' || c == '\r') {
      capturing = false;
      parseThresholds(lineBuf);
      lineBuf = "";
    } else if (lineBuf.length() < 40) {
      lineBuf += c;
    }
    return;
  }
  if (c == 'T') { capturing = true; lineBuf = ""; return; }
  handleCmd(c);
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

  InternalFS.begin();
  loadConfig();

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

  // Seed the filter, then give 3 s to lie flat before a first calibration
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

  while (Serial.available()) handleChar(Serial.read());
  while (bleuart.available()) handleChar(bleuart.read());

  if (ledOffAt && now >= ledOffAt) { ledsOff(); ledOffAt = 0; }

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = now;
    readFiltered();
    float a = angleDeg();
    if (armed) updateReps(a, now);

    if (streamAngle && ++streamDiv >= 5) {   // 10 per second
      streamDiv = 0;
      send("ANG," + String(a, 1));
    }

    if (plotMode) {
      Serial.print("angle:");    Serial.print(a, 1);
      Serial.print(",up:");      Serial.print(cfg.upDeg, 0);
      Serial.print(",partial:"); Serial.print(cfg.partialDeg, 0);
      Serial.print(",down:");    Serial.print(cfg.downDeg, 0);
      Serial.print(",reps:");    Serial.println(reps);
    }
  }
}
