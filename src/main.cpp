/*
 * Fast optical flow on ESP32-CAM (OV2640), camera pointing at the floor.
 * pinout see: https://www.az-delivery.de/en/products/esp32-cam-modul-esp32-wifi-bluetooth-modul-inklusive-kamera
 * manual see: https://cdn.shopify.com/s/files/1/1509/1638/files/ESP32-Cam_datasheet.pdf?v=1689236064
 *
 * Method : 3x3 grid of 24x24 template blocks, SAD block matching (+/-8 px search,
 *          2x subsampled), parabolic sub-pixel refinement, median over valid blocks.
 * Image  : 160x120 8-bit grayscale, both frame buffers kept in internal SRAM.
 * Output : "t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid" per frame on Serial and in /flow.csv
 *          (the 6-column format understood by path_viewer.py)
 *
 * Units / signs: the measured shift of the floor texture in the image (pixels) is converted to
 * millimetres on the floor (MM_PER_PX, see CAM_HEIGHT_MM / CAM_HFOV_DEG) and its sign is inverted,
 * so dx/dy describe the motion of the PLATFORM: +x = right, +y = forward ("up" in the viewer),
 * assuming the top of the camera image points forward. If your camera is mounted differently use
 * "Invert X" / "Invert Y" in the viewer.
 *
 * Serial commands:  p = toggle live print   d = replay (dump) the log   w = delete the log
 *                   r = reset position      x = same as w (old name)
 */

#include <Arduino.h>
#include <math.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include <LittleFS.h>

// ------------------------------------------------------------------ settings
static constexpr int W = 160;               // FRAMESIZE_QQVGA
static constexpr int H = 120;

static constexpr int BS = 24;               // block size
static constexpr int SR = 8;                // search range (+/- px) -> max ~8 px/frame
static constexpr int STEP = 2;              // pixel subsampling inside SAD
static constexpr int NS = 2 * SR + 1;
static constexpr int NBX = 3, NBY = 3;
static const int BX[NBX] = {16, 68, 120};   // block origins (keep SR margin to the border)
static const int BY[NBY] = {12, 48, 84};

static constexpr uint32_t TEX_MIN = 400;                                 // min texture of a block
static constexpr uint32_t MAX_SAD = (BS / STEP) * (BS / STEP) * 25;      // reject bad matches
static constexpr int MIN_VALID_BLOCKS = 3;

// Camera geometry: pixels -> millimetres on the floor (the viewer expects mm).
// This is only a first estimate - CALIBRATE: push the platform a known distance (e.g. 2 m) and
// correct CAM_HEIGHT_MM by (true distance / measured distance).
static constexpr float CAM_HEIGHT_MM = 300.0f;      // distance lens -> floor
static constexpr float CAM_HFOV_DEG = 65.0f;        // horizontal field of view (OV2640 lenses: ~60..70 deg)
static const float MM_PER_PX = 2.0f * CAM_HEIGHT_MM * tanf(CAM_HFOV_DEG * PI / 360.0f) / W;

// Manual exposure (reduces motion blur on a fast platform). 0 = auto exposure.
// Typical OV2640 range 0..1200. Needs enough light on the floor!
#define MANUAL_EXPOSURE 0
#define MANUAL_GAIN 10                      // 0..30, only used with manual exposure

static const char *LOG_PATH = "/flow.csv";
static constexpr size_t LOG_BUF = 4096;
static constexpr uint32_t LOG_FLUSH_MS = 2000;
static constexpr size_t LOG_MAX_BYTES = 1000000;   // stop logging at ~1 MB (partition is ~1.4 MB)

// AI-Thinker / AZ Delivery ESP32-CAM pinout
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ------------------------------------------------------------------ globals
struct FlowSample {
  uint32_t t_ms;
  float dx, dy;     // platform motion since the last frame [mm]
  float x, y;       // accumulated platform motion [mm]
  uint8_t n;        // number of valid blocks
};

static QueueHandle_t flowQueue;
static SemaphoreHandle_t fsMutex;
static volatile bool streamEnabled = true;
static volatile bool loggingEnabled = true;
static volatile bool resetPos = false;
static volatile uint32_t droppedSamples = 0;

// ------------------------------------------------------------------ optical flow
static inline uint32_t sadBlock(const uint8_t *a, const uint8_t *b) {
  uint32_t s = 0;
  for (int y = 0; y < BS; y += STEP) {
    const uint8_t *pa = a + y * W;
    const uint8_t *pb = b + y * W;
    for (int x = 0; x < BS; x += STEP) {
      int d = (int)pa[x] - (int)pb[x];
      s += (d < 0) ? -d : d;
    }
  }
  return s;
}

// Finds where the block at (bx,by) of `prev` moved to in `cur`.
static bool matchBlock(const uint8_t *prev, const uint8_t *cur, int bx, int by,
                       float &ox, float &oy) {
  const uint8_t *tpl = prev + by * W + bx;

  // texture check: flat blocks cannot be tracked
  uint32_t tex = 0;
  for (int y = 0; y < BS - STEP; y += STEP) {
    for (int x = 0; x < BS - STEP; x += STEP) {
      int v = tpl[y * W + x];
      int gx = v - tpl[y * W + x + STEP];
      int gy = v - tpl[(y + STEP) * W + x];
      tex += (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
    }
  }
  if (tex < TEX_MIN) return false;

  uint32_t map[NS][NS];
  uint32_t best = UINT32_MAX, sum = 0;
  int bi = 0, bj = 0;
  for (int j = 0; j < NS; j++) {
    for (int i = 0; i < NS; i++) {
      const uint8_t *c = cur + (by + j - SR) * W + (bx + i - SR);
      uint32_t s = sadBlock(tpl, c);
      map[j][i] = s;
      sum += s;
      if (s < best) { best = s; bi = i; bj = j; }
    }
  }

  float mean = (float)sum / (NS * NS);
  if (best > MAX_SAD || (float)best > 0.8f * mean) return false;   // bad / ambiguous match

  float fx = (float)(bi - SR), fy = (float)(bj - SR);

  // parabolic sub-pixel refinement
  if (bi > 0 && bi < NS - 1) {
    float a = map[bj][bi - 1], b = map[bj][bi], c = map[bj][bi + 1];
    float d = a - 2 * b + c;
    if (d > 0) fx += constrain(0.5f * (a - c) / d, -0.5f, 0.5f);
  }
  if (bj > 0 && bj < NS - 1) {
    float a = map[bj - 1][bi], b = map[bj][bi], c = map[bj + 1][bi];
    float d = a - 2 * b + c;
    if (d > 0) fy += constrain(0.5f * (a - c) / d, -0.5f, 0.5f);
  }
  ox = fx;
  oy = fy;
  return true;
}

static float medianOf(float *v, int n) {
  for (int i = 1; i < n; i++) {
    float k = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static int computeFlow(const uint8_t *prev, const uint8_t *cur, float &dx, float &dy) {
  float vx[NBX * NBY], vy[NBX * NBY];
  int n = 0;
  for (int j = 0; j < NBY; j++) {
    for (int i = 0; i < NBX; i++) {
      float fx, fy;
      if (matchBlock(prev, cur, BX[i], BY[j], fx, fy)) { vx[n] = fx; vy[n] = fy; n++; }
    }
  }
  if (n < MIN_VALID_BLOCKS) { dx = dy = 0; return 0; }
  dx = medianOf(vx, n);
  dy = medianOf(vy, n);
  return n;
}

// ------------------------------------------------------------------ helpers
static int fmtSample(char *out, size_t cap, const FlowSample &s) {
  return snprintf(out, cap, "%lu,%.3f,%.3f,%.2f,%.2f,%u\n", (unsigned long)s.t_ms, s.dx, s.dy,
                  s.x, s.y, s.n);
}

static const char *CSV_HEADER = "# session start; t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid\n";

static void appendToLog(const char *data, size_t len) {
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  File f = LittleFS.open(LOG_PATH, FILE_APPEND);
  if (f) {
    f.write((const uint8_t *)data, len);
    size_t sz = f.size();
    f.close();
    if (sz > LOG_MAX_BYTES && loggingEnabled) {
      loggingEnabled = false;
      Serial.println("# log file full, logging stopped (send 'w' to delete the log)");
    }
  } else {
    Serial.println("# ERROR: cannot open log file");
  }
  xSemaphoreGive(fsMutex);
}

static void dumpLog() {
  bool prev = streamEnabled;
  streamEnabled = false;
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  File f = LittleFS.open(LOG_PATH, FILE_READ);
  Serial.println("# replay 1/1");                                   // only one log file in this version
  Serial.printf("# ---- BEGIN LOG %s ----\n", LOG_PATH + 1);
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
  streamEnabled = prev;
}

static void clearLog() {
  xSemaphoreTake(fsMutex, portMAX_DELAY);
  LittleFS.remove(LOG_PATH);
  xSemaphoreGive(fsMutex);
  appendToLog(CSV_HEADER, strlen(CSV_HEADER));
  loggingEnabled = true;
  Serial.println("# log cleared");
}

// ------------------------------------------------------------------ camera
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_QQVGA;
  c.jpeg_quality = 12;                       // unused for grayscale
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;          // always the newest frame, never a stale one

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("# camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
#if MANUAL_EXPOSURE > 0
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, MANUAL_EXPOSURE);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, MANUAL_GAIN);
#else
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_8X);
#endif
  }
  return true;
}

// ------------------------------------------------------------------ tasks
static void visionTask(void *) {
  uint8_t *prev = (uint8_t *)heap_caps_malloc(W * H, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint8_t *cur = (uint8_t *)heap_caps_malloc(W * H, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!prev || !cur) {
    Serial.println("# out of internal RAM");
    vTaskDelete(NULL);
  }

  bool havePrev = false;
  float accX = 0, accY = 0;
  uint32_t frames = 0, procUs = 0, lastStat = millis();
  char line[72];

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(1); continue; }
    if (fb->width != W || fb->height != H || fb->len != (size_t)(W * H)) {
      esp_camera_fb_return(fb);
      vTaskDelay(1);
      continue;
    }
    uint32_t tMs = (uint32_t)(esp_timer_get_time() / 1000);
    memcpy(cur, fb->buf, W * H);             // PSRAM -> internal SRAM
    esp_camera_fb_return(fb);                // give the buffer back to the driver ASAP

    if (resetPos) { accX = accY = 0; resetPos = false; }

    if (havePrev) {
      int64_t t0 = esp_timer_get_time();
      FlowSample s;
      s.n = (uint8_t)computeFlow(prev, cur, s.dx, s.dy);
      procUs += (uint32_t)(esp_timer_get_time() - t0);
      // floor shift in the image [px] -> platform motion [mm]: the platform moves opposite to the
      // floor, and the image y axis points down while the viewer's y axis points forward/up
      s.dx = -s.dx * MM_PER_PX;
      s.dy = s.dy * MM_PER_PX;
      accX += s.dx;
      accY += s.dy;
      s.t_ms = tMs;
      s.x = accX;
      s.y = accY;

      if (streamEnabled) {
        int n = fmtSample(line, sizeof(line), s);
        Serial.write((const uint8_t *)line, n);
      }
      if (xQueueSend(flowQueue, &s, 0) != pdTRUE) droppedSamples++;
      frames++;
    }
    uint8_t *t = prev; prev = cur; cur = t;
    havePrev = true;

    uint32_t now = millis();
    if (now - lastStat >= 2000) {
      if (streamEnabled)
        Serial.printf("# fps=%.1f flow=%lu us/frame dropped=%lu\n",
                      frames * 1000.0f / (now - lastStat),
                      frames ? (unsigned long)(procUs / frames) : 0UL,
                      (unsigned long)droppedSamples);
      frames = 0; procUs = 0; lastStat = now;
    }
  }
}

static void loggerTask(void *) {
  static char buf[LOG_BUF];
  size_t len = 0;
  uint32_t lastFlush = millis();
  FlowSample s;

  for (;;) {
    if (xQueueReceive(flowQueue, &s, pdMS_TO_TICKS(200)) == pdTRUE && loggingEnabled) {
      if (len + 72 < LOG_BUF) len += fmtSample(buf + len, LOG_BUF - len, s);
    }
    if (len > 0 && (len > LOG_BUF - 160 || millis() - lastFlush >= LOG_FLUSH_MS)) {
      appendToLog(buf, len);
      len = 0;
      lastFlush = millis();
    }
  }
}

// ------------------------------------------------------------------ Arduino
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n# ESP32-CAM optical flow");

  if (!LittleFS.begin(true)) {               // true = format on first use
    Serial.println("# LittleFS mount failed");
  }
  fsMutex = xSemaphoreCreateMutex();
  flowQueue = xQueueCreate(128, sizeof(FlowSample));
  appendToLog(CSV_HEADER, strlen(CSV_HEADER));

  if (!initCamera()) {
    delay(2000);
    ESP.restart();
  }

  xTaskCreatePinnedToCore(visionTask, "vision", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(loggerTask, "logger", 6144, NULL, 1, NULL, 0);
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'd': dumpLog(); break;
      case 'w':
      case 'x': clearLog(); break;                // 'w' is sent by the viewer's "Delete all files"
      case 'r': resetPos = true; Serial.println("# position reset"); break;
      case 'p': streamEnabled = !streamEnabled; break;
      case 'c': Serial.println("# 'c' (centre pots) is not used in the optical flow version"); break;
      default: break;
    }
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}
