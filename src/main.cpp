/*
 * Differential-drive wheel odometry on a generic ESP32 DevKit (30 pin).
 * Pinout see: https://www.robotics.org.za/ESP32-DEV
 *
 * Inputs : two wheel encoders. For now they are SIMULATED by two potentiometers:
 *          wiper position = wheel speed, mid position = standing still.
 *            left  pot wiper -> GPIO35   (ADC1)
 *            right pot wiper -> GPIO34   (ADC1)
 *            pot outer pins  -> 3V3 and GND
 *          Set SIMULATED_ENCODERS to 0 to use real quadrature encoders instead.
 *
 * Output : "t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg" per sample on Serial and in the recording file
 *            dx, dy      = platform displacement since the last sample, in mm
 *            x, y        = accumulated position in mm (start = 0,0, initial heading = +Y = "up")
 *            n_valid     = number of valid wheels (2)
 *            heading_deg = platform heading, 0 = +Y, positive = clockwise (right turn), -180..180
 *          Every second a "# v=.. w=.. hdg=.." comment line (speed, turn rate, heading) is sent.
 *
 * Recording: a short press of the button on GPIO13 (to GND) or the serial command 'p' starts /
 *            pauses the recording (and the live serial output). Every new recording goes to a new
 *            file /rec_001.csv, /rec_002.csv, ... in flash. The internal LED (GPIO2) blinks while
 *            recording. A long press (1 s) or 'd' replays (dumps) the recordings over serial, one
 *            file per press, wrapping around. 'w' deletes all recordings.
 *
 * Serial commands: p = start/pause recording + live output (same as short button press)
 *                  d = replay next recording (same as long button press)
 *                  w = delete all recordings
 *                  r = reset position/heading
 *                  c = set current pot positions as "zero" (simulation)
 *                  s = toggle steering (wheel difference rotates the platform)
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "esp_timer.h"
#include "driver/gpio.h"

// ------------------------------------------------------------------ settings
#define SIMULATED_ENCODERS 0

static constexpr int SAMPLE_HZ = 20;                // odometry / output rate

// Mechanics (adjust to your platform)
// Wheelchair - these are placeholders, MEASURE YOURS.
// The encoders are pressed against the tyres, so the distance per count is set by the diameter
// of the small measuring roller on the encoder shaft (not by the wheelchair wheel), assuming no slip.
static constexpr float WHEEL_DIAMETER_MM = 56.5f;   // diameter of the encoder roller
static constexpr float TRACK_WIDTH_MM = 560.0f;     // distance between the two wheelchair wheel contact points

// --- simulation (potentiometers)
static constexpr int POT_L_PIN = 35;
static constexpr int POT_R_PIN = 34;
static constexpr float SIM_MAX_SPEED_MM_S = 1000.0f; // wheel speed at full pot deflection
static constexpr int POT_DEADBAND = 80;             // ADC counts around mid = no movement
static constexpr int POT_HALF_RANGE = 1900;         // ADC counts from mid to "full speed"
static int potMidL = 2048, potMidR = 2048;          // can be re-centred with the 'c' command

// --- real encoders (quadrature, 4x decoding)
static constexpr int ENCODER_PPR = 600; //360;             // pulses per revolution per channel
static constexpr int ENC_R_A = 14, ENC_R_B = 27;
static constexpr int ENC_L_A = 25, ENC_L_B = 26;

static constexpr int ENC_LEFT_SIGN = 1;             // set to -1 if a wheel counts backwards when driving forward
static constexpr int ENC_RIGHT_SIGN = 1;            // (the right encoder is usually mirrored -> -1)
static constexpr float MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / (4.0f * ENCODER_PPR);
// Right wheel distance is multiplied by this. Tune on a straight drive: path drifts right -> increase,
// drifts left -> decrease. (1.0 = no compensation; e.g. 1.02 if the right encoder counts 2% too few)
static constexpr float RIGHT_WHEEL_COMP = 1.020f;

// --- button and LED
static constexpr int BTN_PIN = 13;                  // push button between GPIO13 and GND (internal pull-up)
static constexpr int LED_PIN = 2;                   // internal LED of the ESP32 DevKit
static constexpr uint32_t DEBOUNCE_MS = 40;
static constexpr uint32_t LONG_PRESS_MS = 1000;     // press longer than this = replay
static constexpr uint32_t BLINK_MS = 250;           // LED toggles every 250 ms while recording

// --- recording
static constexpr size_t LOG_BUF = 4096;
static constexpr uint32_t LOG_FLUSH_MS = 2000;
static constexpr size_t LOG_MAX_BYTES = 1000000;    // one recording stops at ~1 MB
static constexpr size_t MIN_FREE_BYTES = 65536;     // do not start a recording with less free flash

// ------------------------------------------------------------------ globals
struct OdoSample {
  uint32_t t_ms;
  float dx, dy;     // displacement since last sample [mm]
  float x, y;       // accumulated position [mm]
  uint8_t n;        // number of valid wheels
  float h;          // heading [deg], 0 = +Y, clockwise positive, -180..180
};

static QueueHandle_t sampleQueue;
static SemaphoreHandle_t fsMutex;
static volatile bool streamEnabled = true;
static volatile bool recording = false;      // file recording active (LED blinks)
static volatile bool loggerPending = false;  // logger task still holds unwritten samples
static char recPath[24] = "";                // file of the current / last recording
static volatile uint32_t recSession = 0;     // incremented for every new recording
static int replayIdx = 0;                    // next recording to replay
static volatile bool resetPos = false;
static volatile bool calibrateReq = false;
static volatile uint32_t droppedSamples = 0;

// Steering = let the DIFFERENCE between the two wheels rotate the platform.
// ON by default. Press 's' to switch it off: the platform then only drives straight along +Y/-Y
// with the average of both wheels, which is handy to check the signs of the pots.
static volatile bool steering = true;

// debug values shown in the status line (raw ADC value of the pots / encoder counts)
static volatile int dbgRawL = 0, dbgRawR = 0;

// ------------------------------------------------------------------ wheel input
#if SIMULATED_ENCODERS

static int readAveraged(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(pin);
  return 4096-(int)(acc / 16);
}

// raw ADC value -> -1..+1 with dead band around the mid position
static float potToNorm(int raw, int mid) {
  int d = raw - mid;
  if (abs(d) <= POT_DEADBAND) return 0.0f;
  d += (d > 0) ? -POT_DEADBAND : POT_DEADBAND;
  float n = (float)d / (float)(POT_HALF_RANGE - POT_DEADBAND);
  return constrain(n, -1.0f, 1.0f);
}

static void initWheelInput() {
  analogReadResolution(12);
  analogSetPinAttenuation(POT_L_PIN, ADC_11db);
  analogSetPinAttenuation(POT_R_PIN, ADC_11db);
  Serial.printf("# SIMULATED encoders: pots on GPIO%d (left) / GPIO%d (right)\n", POT_L_PIN, POT_R_PIN);
}

static void calibrateInput() {
  potMidL = readAveraged(POT_L_PIN);
  potMidR = readAveraged(POT_R_PIN);
  Serial.printf("# pot centre set: L=%d R=%d\n", potMidL, potMidR);
}

// distance travelled by each wheel during the last dt seconds [mm]
static void readWheelDeltas(float dt, float &dL, float &dR) {
  dbgRawL = readAveraged(POT_L_PIN);
  dbgRawR = readAveraged(POT_R_PIN);
  dL = potToNorm(dbgRawL, potMidL) * SIM_MAX_SPEED_MM_S * dt;
  dR = potToNorm(dbgRawR, potMidR) * SIM_MAX_SPEED_MM_S * dt;
}

#else  // real quadrature encoders

struct Quad {
  uint8_t pinA, pinB;
  volatile int32_t count;
  volatile uint8_t state;
};
static Quad encL = {ENC_L_A, ENC_L_B, 0};
static Quad encR = {ENC_R_A, ENC_R_B, 0};

// index = (previous state << 2) | new state, state = (A << 1) | B
static const DRAM_ATTR int8_t QUAD_TABLE[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static void IRAM_ATTR quadISR(void *arg) {
  Quad *q = (Quad *)arg;
  uint8_t s = (uint8_t)((gpio_get_level((gpio_num_t)q->pinA) << 1) | gpio_get_level((gpio_num_t)q->pinB));
  q->count += QUAD_TABLE[(q->state << 2) | s];
  q->state = s;
}

static void attachQuad(Quad &q) {
  pinMode(q.pinA, INPUT);  // note: use external pullups to 3V3!
  pinMode(q.pinB, INPUT);
  q.state = (uint8_t)((digitalRead(q.pinA) << 1) | digitalRead(q.pinB));
  attachInterruptArg(q.pinA, quadISR, &q, CHANGE);
  attachInterruptArg(q.pinB, quadISR, &q, CHANGE);
}

static void initWheelInput() {
  attachQuad(encL);
  attachQuad(encR);
  Serial.println("# REAL encoders enabled");
}

static void calibrateInput() {}

static void readWheelDeltas(float dt, float &dL, float &dR) {
  (void)dt;
  static int32_t lastL = 0, lastR = 0;
  int32_t cl = encL.count, cr = encR.count;
  dbgRawL = cl;
  dbgRawR = cr;
  dL = ENC_LEFT_SIGN * (float)(cl - lastL) * MM_PER_COUNT;
  dR = ENC_RIGHT_SIGN * (float)(cr - lastR) * MM_PER_COUNT * RIGHT_WHEEL_COMP;
  lastL = cl;
  lastR = cr;
}

#endif

// ------------------------------------------------------------------ helpers
static int fmtSample(char *out, size_t cap, const OdoSample &s) {
  return snprintf(out, cap, "%lu,%.3f,%.3f,%.2f,%.2f,%u,%.2f\n", (unsigned long)s.t_ms, s.dx,
                  s.dy, s.x, s.y, s.n, s.h);
}

static const char *CSV_HEADER = "# recording; t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg\n";

static bool isRecFile(const String &n) { return n.startsWith("rec_") && n.endsWith(".csv"); }

// Sorted names of all recordings (without leading '/'). Caller must hold fsMutex.
static std::vector<String> listRecordings() {
  std::vector<String> v;
  File root = LittleFS.open("/");
  if (!root) return v;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    String n = f.name();
    if (n.startsWith("/")) n = n.substring(1);
    if (!f.isDirectory() && isRecFile(n)) v.push_back(n);
  }
  root.close();
  std::sort(v.begin(), v.end());
  return v;
}

// Append samples to a recording file (called by the logger task only).
static void flushLog(const char *path, const char *data, size_t len) {
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  bool ok = false;
  size_t sz = 0;
  File f = LittleFS.open(path, FILE_APPEND);
  if (f) {
    ok = f.write((const uint8_t *)data, len) == len;
    sz = f.size();
    f.close();
  }
  xSemaphoreGive(fsMutex);

  if (!ok) {
    recording = false;
    Serial.println("# ERROR: cannot write file (flash full?) - recording stopped, send 'w' to delete recordings");
  } else if (sz > LOG_MAX_BYTES && recording) {
    recording = false;
    Serial.println("# file full - recording stopped");
  }
}

static void startRecording() {
  if (recording) return;
  xSemaphoreTake(fsMutex, portMAX_DELAY);

  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  if (freeBytes < MIN_FREE_BYTES) {
    xSemaphoreGive(fsMutex);
    Serial.println("# flash full - cannot start, send 'w' to delete all recordings");
    return;
  }

  int maxN = 0;                                       // new file = highest number + 1
  for (const String &n : listRecordings()) {
    int k = 0;
    if (sscanf(n.c_str(), "rec_%d.csv", &k) == 1 && k > maxN) maxN = k;
  }
  char path[24];
  snprintf(path, sizeof(path), "/rec_%03d.csv", maxN + 1);

  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) {
    xSemaphoreGive(fsMutex);
    Serial.printf("# ERROR: cannot create %s\n", path);
    return;
  }
  f.print(CSV_HEADER);
  f.close();
  strlcpy(recPath, path, sizeof(recPath));
  recSession++;
  xSemaphoreGive(fsMutex);

  xQueueReset(sampleQueue);                           // drop samples from before the start
  streamEnabled = true;
  recording = true;
  Serial.printf("# recording started: %s\n", path);
}

static void stopRecording() {
  if (!recording) return;
  recording = false;
  uint32_t t0 = millis();                             // wait until the logger has written everything
  while (loggerPending && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(10));

  xSemaphoreTake(fsMutex, portMAX_DELAY);
  File f = LittleFS.open(recPath, FILE_READ);
  size_t sz = f ? f.size() : 0;
  if (f) f.close();
  xSemaphoreGive(fsMutex);
  Serial.printf("# recording stopped: %s (%u bytes)\n", recPath, (unsigned)sz);
}

// short button press and 'p' command
static void toggleRecording() {
  if (recording) {
    stopRecording();
    streamEnabled = false;
  } else {
    startRecording();
  }
}

static void dumpFile(const String &name) {
  bool prev = streamEnabled;
  streamEnabled = false;
  digitalWrite(LED_PIN, HIGH);                        // LED on while dumping
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  File f = LittleFS.open("/" + name, FILE_READ);
  Serial.printf("# ---- BEGIN LOG %s ----\n", name.c_str());
  if (f) {
    uint8_t tmp[256];
    while (f.available()) {
      int n = f.read(tmp, sizeof(tmp));
      if (n <= 0) break;
      Serial.write(tmp, n);
    }
    f.close();
  }
  Serial.println("# ---- END LOG ----");
  xSemaphoreGive(fsMutex);
  digitalWrite(LED_PIN, LOW);
  streamEnabled = prev;
}

// long button press and 'd' command: dump the next recording, wrap around after the last one
static void replayNext() {
  if (recording) {
    Serial.println("# stop the recording first (p / short press) before replaying");
    return;
  }
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  std::vector<String> files = listRecordings();
  xSemaphoreGive(fsMutex);
  if (files.empty()) {
    Serial.println("# no recordings");
    return;
  }
  int idx = replayIdx % (int)files.size();
  replayIdx = idx + 1;
  Serial.printf("# replay %d/%d\n", idx + 1, (int)files.size());
  dumpFile(files[idx]);
}

// 'w' command
static void wipeRecordings() {
  if (recording) {
    Serial.println("# stop the recording first (p / short press) before deleting");
    return;
  }
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  int n = 0;
  for (const String &name : listRecordings())
    if (LittleFS.remove("/" + name)) n++;
  LittleFS.remove("/flow.csv");                       // file of older firmware versions
  xSemaphoreGive(fsMutex);
  replayIdx = 0;
  Serial.printf("# %d recording(s) deleted\n", n);
}

// Button: short press (release before LONG_PRESS_MS) = start/pause, long press = replay next file
static void handleButton() {
  static bool down = false, longDone = false;
  static uint32_t t0 = 0;
  bool pressed = digitalRead(BTN_PIN) == LOW;
  uint32_t now = millis();

  if (pressed && !down) {
    down = true;
    longDone = false;
    t0 = now;
  } else if (pressed && down) {
    if (!longDone && now - t0 >= LONG_PRESS_MS) {
      longDone = true;
      replayNext();
    }
  } else if (!pressed && down) {
    down = false;
    if (!longDone && now - t0 >= DEBOUNCE_MS) toggleRecording();
  }
}

// LED: blinking while recording, otherwise off
static void updateLed() {
  static uint32_t last = 0;
  static bool on = false;
  if (!recording) {
    if (on) { digitalWrite(LED_PIN, LOW); on = false; }
    return;
  }
  uint32_t now = millis();
  if (now - last >= BLINK_MS) {
    last = now;
    on = !on;
    digitalWrite(LED_PIN, on ? HIGH : LOW);
  }
}

// ------------------------------------------------------------------ tasks
// Differential-drive odometry. Heading th is measured clockwise from +Y ("up" in the viewer).
static void odometryTask(void *) {
  float x = 0, y = 0, th = 0;
  float v = 0;                                     // for the status line
  const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
  TickType_t wake = xTaskGetTickCount();
  int64_t tPrev = esp_timer_get_time();
  uint32_t lastStat = millis();
  char line[72];

  for (;;) {
    vTaskDelayUntil(&wake, period);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - tPrev) * 1e-6f;
    tPrev = now;

    if (calibrateReq) { calibrateInput(); calibrateReq = false; }
    if (resetPos) { x = y = th = 0; resetPos = false; }

    float dL, dR;
    readWheelDeltas(dt, dL, dR);

    float ds = 0.5f * (dL + dR);                   // distance travelled by the platform centre
    float dth = steering ? (dL - dR) / TRACK_WIDTH_MM : 0.0f;   // left faster than right -> turn clockwise
    float mid = th + 0.5f * dth;                   // midpoint integration
    float dx = ds * sinf(mid);
    float dy = ds * cosf(mid);
    x += dx;
    y += dy;
    th += dth;
    if (dt > 0) v = ds / dt;

    float hdg = fmodf(th * 180.0f / PI, 360.0f);
    if (hdg > 180.0f) hdg -= 360.0f;
    if (hdg < -180.0f) hdg += 360.0f;

    OdoSample s;
    s.t_ms = (uint32_t)(now / 1000);
    s.dx = dx;
    s.dy = dy;
    s.x = x;
    s.y = y;
    s.n = 2;
    s.h = hdg;

    if (streamEnabled) {
      int n = fmtSample(line, sizeof(line), s);
      Serial.write((const uint8_t *)line, n);
    }
    if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) droppedSamples++;

    uint32_t ms = millis();
    if (ms - lastStat >= 500) {
      lastStat = ms;
      float vL = dt > 0 ? dL / dt : 0, vR = dt > 0 ? dR / dt : 0;
      if (streamEnabled)
        Serial.printf("# L raw=%d %.0f mm/s | R raw=%d %.0f mm/s | v=%.0f mm/s hdg=%.1f deg steer=%s rec=%s\n",
                      (int)dbgRawL, vL, (int)dbgRawR, vR, v, hdg, steering ? "on" : "off",
                      recording ? "on" : "off");
    }
  }
}

static void loggerTask(void *) {
  static char buf[LOG_BUF];
  size_t len = 0;
  char myPath[24] = "";
  uint32_t mySession = 0;
  uint32_t lastFlush = millis();
  OdoSample s;

  for (;;) {
    bool got = xQueueReceive(sampleQueue, &s, pdMS_TO_TICKS(50)) == pdTRUE;

    // a new recording was started: finish the old buffer, then switch to the new file
    xSemaphoreTake(fsMutex, portMAX_DELAY);
    uint32_t sess = recSession;
    char newPath[24];
    strlcpy(newPath, recPath, sizeof(newPath));
    xSemaphoreGive(fsMutex);
    if (sess != mySession) {
      if (len > 0 && myPath[0]) flushLog(myPath, buf, len);
      len = 0;
      strlcpy(myPath, newPath, sizeof(myPath));
      mySession = sess;
      lastFlush = millis();
    }

    if (got && recording && len + 72 < LOG_BUF) len += fmtSample(buf + len, LOG_BUF - len, s);
    loggerPending = len > 0;

    // write when the buffer is full, every LOG_FLUSH_MS, or immediately after the recording stopped
    if (len > 0 && (!recording || len > LOG_BUF - 160 || millis() - lastFlush >= LOG_FLUSH_MS)) {
      flushLog(myPath, buf, len);
      len = 0;
      loggerPending = false;
      lastFlush = millis();
    }
  }
}

// ------------------------------------------------------------------ Arduino
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n# ESP32 wheel odometry");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BTN_PIN, INPUT_PULLUP);

  if (!LittleFS.begin(true)) {               // true = format on first use
    Serial.println("# LittleFS mount failed");
  }
  fsMutex = xSemaphoreCreateMutex();
  sampleQueue = xQueueCreate(128, sizeof(OdoSample));

  xSemaphoreTake(fsMutex, portMAX_DELAY);
  size_t nFiles = listRecordings().size();
  xSemaphoreGive(fsMutex);
  Serial.printf("# %u recording(s) in flash. Short press / 'p' = start/pause, long press / 'd' = replay, 'w' = delete all\n",
                (unsigned)nFiles);

  initWheelInput();

  xTaskCreatePinnedToCore(odometryTask, "odometry", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(loggerTask, "logger", 6144, NULL, 1, NULL, 0);
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': toggleRecording(); break;
      case 'd': replayNext(); break;
      case 'w': wipeRecordings(); break;
      case 'r': resetPos = true; Serial.println("# position reset"); break;
      case 'c': calibrateReq = true; break;
      case 's': steering = !steering; Serial.printf("# steering %s\n", steering ? "on" : "off"); break;
      default: break;
    }
  }
  handleButton();
  updateLed();
  vTaskDelay(pdMS_TO_TICKS(20));
}
