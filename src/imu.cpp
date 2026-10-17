#include "imu.h"
#include <Wire.h>
#include <math.h>
#include "esp_timer.h"
#include "config.h"

namespace {

// MPU-6050 registers
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // 14 bytes: ax ay az temp gx gy gz (big endian)
constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float GYRO_LSB_PER_DPS = 131.0f;   // full scale +-250 dps
constexpr float ACCEL_LSB_PER_G = 8192.0f;   // full scale +-4 g
constexpr float MAX_DT_S = 0.1f;             // longest time step used for integration (flash writes can stall the task)
constexpr int FAIL_LIMIT = 20;               // consecutive I2C errors before the IMU is declared lost

struct Raw {
  float g[3];    // gyro [deg/s]
  float a[3];    // accelerometer [g]
  float tempC;
};

// ---- state shared with other tasks
volatile ImuState gState = ImuState::Off;
volatile bool gHealthy = false;
volatile bool gRecalReq = false;
volatile bool gWheelsStill = false;
volatile float gRateDps = 0, gBiasDps = 0, gTempC = 0;
float gYawAccum = 0;                          // [rad] clockwise, guarded by gMux
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

// ---- state of the sampling task
float gBias[3] = {0, 0, 0};                   // gyro zero-rate offset per sensor axis [deg/s]
float gUp[3] = {0, 0, 1};                     // low-passed accelerometer = direction of "up" in sensor axes [g]
float gRateFilt = 0;

struct CalWindow {
  int n;
  double sg[3], sa[3];
  float gmin[3], gmax[3], amin, amax;
  void reset() {
    n = 0;
    for (int i = 0; i < 3; i++) { sg[i] = sa[i] = 0; gmin[i] = 1e9f; gmax[i] = -1e9f; }
    amin = 1e9f; amax = -1e9f;
  }
  void add(const Raw &r) {
    float am = sqrtf(r.a[0] * r.a[0] + r.a[1] * r.a[1] + r.a[2] * r.a[2]);
    for (int i = 0; i < 3; i++) {
      sg[i] += r.g[i];
      sa[i] += r.a[i];
      gmin[i] = fminf(gmin[i], r.g[i]);
      gmax[i] = fmaxf(gmax[i], r.g[i]);
    }
    amin = fminf(amin, am);
    amax = fmaxf(amax, am);
    n++;
  }
} gCal;

// ---- I2C helpers (used by the sampling task only, after imuBegin)
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(cfg::IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(cfg::IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)cfg::IMU_ADDR, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool readSample(Raw &r) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  r.a[0] = s16(0) / ACCEL_LSB_PER_G;
  r.a[1] = s16(2) / ACCEL_LSB_PER_G;
  r.a[2] = s16(4) / ACCEL_LSB_PER_G;
  r.tempC = s16(6) / 340.0f + 36.53f;
  r.g[0] = s16(8) / GYRO_LSB_PER_DPS;
  r.g[1] = s16(10) / GYRO_LSB_PER_DPS;
  r.g[2] = s16(12) / GYRO_LSB_PER_DPS;
  return true;
}

// ---- calibration
void startCalibration() {
  gCal.reset();
  portENTER_CRITICAL(&gMux);
  gYawAccum = 0;
  portEXIT_CRITICAL(&gMux);
  gState = ImuState::Calibrating;
}

void calibrationStep(const Raw &r) {
  gCal.add(r);
  const int needed = cfg::IMU_CAL_MS * cfg::IMU_HZ / 1000;
  if (gCal.n < needed) return;

  float spreadG = 0;
  for (int i = 0; i < 3; i++) spreadG = fmaxf(spreadG, gCal.gmax[i] - gCal.gmin[i]);
  float spreadA = gCal.amax - gCal.amin;
  if (spreadG > cfg::IMU_CAL_MAX_SPREAD_DPS || spreadA > cfg::IMU_CAL_MAX_ACC_SPREAD_G) {
    Serial.printf("# gyro calibration: chair is moving (gyro %.2f dps, accel %.3f g spread) - retrying, keep it still\n",
                  spreadG, spreadA);
    gCal.reset();
    return;
  }

  for (int i = 0; i < 3; i++) {
    gBias[i] = (float)(gCal.sg[i] / gCal.n);
    gUp[i] = (float)(gCal.sa[i] / gCal.n);
  }
  gRateFilt = 0;
  float gn = sqrtf(gUp[0] * gUp[0] + gUp[1] * gUp[1] + gUp[2] * gUp[2]);
  float tilt = gn > 0.1f ? acosf(fminf(1.0f, fabsf(gUp[2]) / gn)) * RAD_TO_DEG : 0.0f;
  Serial.printf("# gyro calibrated: offset x=%.3f y=%.3f z=%.3f dps, |g|=%.3f, board tilt %.1f deg, %.1f C\n",
                gBias[0], gBias[1], gBias[2], gn, tilt, r.tempC);
  if (gn < 0.8f || gn > 1.2f) Serial.println("# WARNING: accelerometer does not read 1 g - check the sensor");
  gState = ImuState::Running;
}

// ---- normal operation
void runStep(const Raw &r, float dt) {
  // direction of "up": slow low-pass of the accelerometer
  float kG = dt / cfg::IMU_GRAVITY_TAU_S;
  for (int i = 0; i < 3; i++) gUp[i] += kG * (r.a[i] - gUp[i]);

  float w[3];
  float wmag;
  auto correct = [&]() {
    for (int i = 0; i < 3; i++) w[i] = r.g[i] - gBias[i];
    wmag = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  };
  correct();

  // standing still: re-learn the zero-rate offset (it drifts with temperature)
  if (gWheelsStill && wmag < cfg::IMU_BIAS_GATE_DPS) {
    float kB = dt / cfg::IMU_BIAS_TAU_S;
    for (int i = 0; i < 3; i++) gBias[i] += kB * (r.g[i] - gBias[i]);
    correct();
  }

  float gn = sqrtf(gUp[0] * gUp[0] + gUp[1] * gUp[1] + gUp[2] * gUp[2]);
  if (gn < 0.3f) return;                                   // no sensible gravity reading
  float ux = gUp[0] / gn, uy = gUp[1] / gn, uz = gUp[2] / gn;
  float up = w[0] * ux + w[1] * uy + w[2] * uz;            // rate about "up", counter-clockwise positive
  float rateCW = (cfg::IMU_INVERT_YAW ? up : -up) * cfg::IMU_YAW_SCALE;   // heading is clockwise positive

  gRateFilt += 0.05f * (rateCW - gRateFilt);
  gRateDps = gRateFilt;
  gBiasDps = (cfg::IMU_INVERT_YAW ? 1.0f : -1.0f) * (gBias[0] * ux + gBias[1] * uy + gBias[2] * uz);

  // standing still and no rotation: freeze the heading so noise cannot accumulate
  bool freeze = gWheelsStill && wmag < cfg::IMU_FREEZE_DPS;
  if (!freeze) {
    portENTER_CRITICAL(&gMux);
    gYawAccum += rateCW * dt * DEG_TO_RAD;
    portEXIT_CRITICAL(&gMux);
  }
}

void imuTask(void *) {
  TickType_t period = pdMS_TO_TICKS(1000 / cfg::IMU_HZ);
  if (period < 1) period = 1;
  TickType_t wake = xTaskGetTickCount();
  int64_t tPrev = esp_timer_get_time();
  int failStreak = 0;

  for (;;) {
    vTaskDelayUntil(&wake, period);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - tPrev) * 1e-6f;
    tPrev = now;
    if (dt > MAX_DT_S) dt = MAX_DT_S;
    if (dt <= 0) continue;

    Raw r;
    if (!readSample(r)) {
      if (++failStreak == FAIL_LIMIT) {
        gHealthy = false;
        Serial.println("# IMU read errors - heading from the encoders until it answers again");
      }
      continue;
    }
    if (failStreak >= FAIL_LIMIT) Serial.println("# IMU is back");
    failStreak = 0;
    gHealthy = true;
    gTempC = r.tempC;

    if (gRecalReq) {
      gRecalReq = false;
      startCalibration();
      Serial.println("# gyro calibration started - keep the chair still");
    }
    if (gState == ImuState::Calibrating) calibrationStep(r);
    else if (gState == ImuState::Running) runStep(r, dt);
  }
}

}  // namespace

// ------------------------------------------------------------------ public API
bool imuBegin() {
  Wire.begin(cfg::IMU_SDA, cfg::IMU_SCL, cfg::IMU_I2C_HZ);
  Wire.setTimeOut(10);

  uint8_t who = 0;
  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    Serial.printf("# IMU not found at 0x%02X (SDA=GPIO%d, SCL=GPIO%d) - heading from the encoders only\n",
                  cfg::IMU_ADDR, cfg::IMU_SDA, cfg::IMU_SCL);
    gState = ImuState::Off;
    return false;
  }
  if (who != 0x68) Serial.printf("# IMU WHO_AM_I = 0x%02X (expected 0x68, clone?) - trying anyway\n", who);

  writeReg(REG_PWR_MGMT_1, 0x80);                 // device reset
  delay(100);
  writeReg(REG_SIGNAL_PATH_RESET, 0x07);
  delay(100);
  writeReg(REG_PWR_MGMT_1, 0x01);                 // wake up, clock = gyro X PLL
  delay(10);
  writeReg(REG_SMPLRT_DIV, 0);                    // 1 kHz internal rate, we poll the newest value at IMU_HZ
  writeReg(REG_CONFIG, 0x03);                     // digital low-pass 42 Hz (rejects vibration)
  writeReg(REG_GYRO_CONFIG, 0x00);                // +-250 deg/s
  writeReg(REG_ACCEL_CONFIG, 0x08);               // +-4 g
  delay(50);

  startCalibration();
  gHealthy = true;
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, NULL, 4, NULL, 1);
  return true;
}

ImuState imuState() { return gState; }

const char *imuStateName() {
  switch (gState) {
    case ImuState::Off: return "off";
    case ImuState::Calibrating: return "calibrating";
    default: return gHealthy ? "ok" : "lost";
  }
}

bool imuUsable() { return gState == ImuState::Running && gHealthy; }

bool imuWaitReady(uint32_t timeoutMs, void (*idle)()) {
  uint32_t t0 = millis();
  while (gState == ImuState::Calibrating && millis() - t0 < timeoutMs) {
    if (idle) idle();
    delay(10);
  }
  return gState == ImuState::Running;
}

void imuRecalibrate() {
  if (gState != ImuState::Off) gRecalReq = true;
  else Serial.println("# no IMU - nothing to calibrate");
}

void imuSetWheelsStill(bool still) { gWheelsStill = still; }

float imuTakeYaw() {
  portENTER_CRITICAL(&gMux);
  float v = gYawAccum;
  gYawAccum = 0;
  portEXIT_CRITICAL(&gMux);
  return v;
}

float imuYawRateDps() { return gRateDps; }
float imuBiasDps() { return gBiasDps; }
float imuTempC() { return gTempC; }
