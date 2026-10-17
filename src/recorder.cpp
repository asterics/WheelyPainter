#include "recorder.h"
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include "config.h"

namespace {

QueueHandle_t sampleQueue;
SemaphoreHandle_t fsMutex;
volatile bool streamEnabled = true;
volatile bool recording = false;       // file recording active (LED blinks)
volatile bool loggerPending = false;   // logger task still holds unwritten samples
char recPath[24] = "";                 // file of the current / last recording
volatile uint32_t recSession = 0;      // incremented for every new recording
int replayIdx = 0;                     // next recording to replay
volatile uint32_t droppedSamples = 0;

bool isRecFile(const String &n) { return n.startsWith("rec_") && n.endsWith(".csv"); }

// Sorted names of all recordings (without leading '/'). Caller must hold fsMutex.
std::vector<String> listRecordings() {
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
void flushLog(const char *path, const char *data, size_t len) {
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
  } else if (sz > cfg::LOG_MAX_BYTES && recording) {
    recording = false;
    Serial.println("# file full - recording stopped");
  }
}

void startRecording() {
  if (recording) return;
  xSemaphoreTake(fsMutex, portMAX_DELAY);

  size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  if (freeBytes < cfg::MIN_FREE_BYTES) {
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

void stopRecording() {
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

void dumpFile(const String &name) {
  bool prev = streamEnabled;
  streamEnabled = false;
  digitalWrite(cfg::LED_PIN, HIGH);                   // LED on while dumping
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
  digitalWrite(cfg::LED_PIN, LOW);
  streamEnabled = prev;
}

void loggerTask(void *) {
  static char buf[cfg::LOG_BUF];
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

    if (got && recording && len + 72 < cfg::LOG_BUF) len += fmtSample(buf + len, cfg::LOG_BUF - len, s);
    loggerPending = len > 0;

    // write when the buffer is full, every LOG_FLUSH_MS, or immediately after the recording stopped
    if (len > 0 && (!recording || len > cfg::LOG_BUF - 160 || millis() - lastFlush >= cfg::LOG_FLUSH_MS)) {
      flushLog(myPath, buf, len);
      len = 0;
      loggerPending = false;
      lastFlush = millis();
    }
  }
}

}  // namespace

// ------------------------------------------------------------------ public API
bool recorderBegin() {
  bool ok = LittleFS.begin(true);            // true = format on first use
  if (!ok) Serial.println("# LittleFS mount failed");
  fsMutex = xSemaphoreCreateMutex();
  sampleQueue = xQueueCreate(128, sizeof(OdoSample));

  xSemaphoreTake(fsMutex, portMAX_DELAY);
  size_t nFiles = listRecordings().size();
  xSemaphoreGive(fsMutex);
  Serial.printf("# %u recording(s) in flash\n", (unsigned)nFiles);

  xTaskCreatePinnedToCore(loggerTask, "logger", 6144, NULL, 1, NULL, 0);
  return ok;
}

void recorderSubmit(const OdoSample &s) {
  if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) droppedSamples++;
}

bool recorderIsRecording() { return recording; }
bool recorderStreamEnabled() { return streamEnabled; }

void recorderToggle() {
  if (recording) {
    stopRecording();
    streamEnabled = false;
  } else {
    startRecording();
  }
}

void recorderReplayNext() {
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

void recorderWipe() {
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
