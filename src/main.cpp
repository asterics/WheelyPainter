/*
 * Wheelchair odometry on a generic ESP32 DevKit (30 pin): two wheel encoders + GY-521 (MPU-6050) gyro.
 * Pinout see: https://www.robotics.org.za/ESP32-DEV
 *
 * Distance comes from the wheel encoders. The heading comes from the gyro, so that slipping
 * encoder rollers no longer destroy it. If the IMU is missing the heading falls back to the encoders.
 *
 * Output : "t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg" per sample on Serial and in the recording file
 *            dx, dy      = platform displacement since the last sample, in mm
 *            x, y        = accumulated position in mm (start = 0,0, initial heading = +Y = "up")
 *            n_valid     = number of valid wheels (always 2)
 *            heading_deg = platform heading, 0 = +Y, positive = clockwise (right turn), -180..180
 *          Twice a second a "# status ..." comment line with diagnostics is sent.
 *
 * Recording: a short press of the button on GPIO13 (to GND) or the serial command 'p' starts / pauses
 *            the recording (and the live serial output). Every new recording goes to a new file
 *            /rec_001.csv, /rec_002.csv, ... in flash. The internal LED (GPIO2) blinks while recording
 *            (fast while the gyro calibrates). A long press (1 s) or 'd' replays one recording over serial.
 *
 * Serial commands: p = start/pause recording + live output (same as short button press)
 *                  d = replay next recording (same as long button press)
 *                  w = delete all recordings
 *                  r = reset position/heading
 *                  g = calibrate the gyro offset (chair must stand still for 2 s)
 *                  h = toggle heading source: gyro (default) / encoders only (for comparison)
 *
 * Modules: config.h (all settings), encoders, imu (gyro driver + calibration), odometry (fusion),
 *          recorder (flash recording, replay), ui (button + LED).
 */

#include <Arduino.h>
#include "esp_timer.h"
#include "config.h"
#include "sample.h"
#include "encoders.h"
#include "imu.h"
#include "odometry.h"
#include "recorder.h"
#include "ui.h"

static volatile bool resetReq = false;
static volatile bool gyroPreferred = true;

// 20 Hz: read encoders + gyro, update the position, output the sample.
static void odometryTask(void *) {
  Odometry odo;
  const TickType_t period = pdMS_TO_TICKS(1000 / cfg::SAMPLE_HZ);
  TickType_t wake = xTaskGetTickCount();
  int64_t tPrev = esp_timer_get_time();
  uint32_t lastStat = millis();
  char line[72];

  for (;;) {
    vTaskDelayUntil(&wake, period);
    int64_t now = esp_timer_get_time();
    float dt = (float)(now - tPrev) * 1e-6f;
    tPrev = now;

    if (resetReq) { odo.reset(); resetReq = false; }
    odo.setPreferGyro(gyroPreferred);

    float dL, dR;
    encodersReadDeltas(dL, dR);
    float dthGyro = imuTakeYaw();                         // always drain, even if not used
    Odometry::Step st = odo.update(dL, dR, dthGyro, imuUsable(), dt);
    imuSetWheelsStill(odo.wheelsStill());

    OdoSample s;
    s.t_ms = (uint32_t)(now / 1000);
    s.dx = st.dx;
    s.dy = st.dy;
    s.x = odo.x();
    s.y = odo.y();
    s.n = 2;
    s.h = odo.headingDeg();

    if (recorderStreamEnabled()) {
      int n = fmtSample(line, sizeof(line), s);
      Serial.write((const uint8_t *)line, n);
    }
    recorderSubmit(s);

    uint32_t ms = millis();
    if (ms - lastStat >= 500) {
      lastStat = ms;
      if (recorderStreamEnabled())
        Serial.printf("# status L=%ld R=%ld v=%.0f mm/s hdg=%.1f deg | gyro %.1f dps, enc_hdg=%.1f gyro_hdg=%.1f | "
                      "imu=%s bias=%.3f dps %.1f C slip=%lu src=%s still=%d rec=%s\n",
                      (long)encoderCountL(), (long)encoderCountR(), odo.speed(), odo.headingDeg(),
                      imuYawRateDps(), odo.encHeadingDeg(), odo.gyroHeadingDeg(), imuStateName(),
                      imuBiasDps(), imuTempC(), (unsigned long)odo.slipSamples(),
                      odo.usingGyro() ? "gyro" : "enc", odo.wheelsStill() ? 1 : 0,
                      recorderIsRecording() ? "on" : "off");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n# ESP32 wheelchair odometry (encoders + gyro)");

  uiBegin();
  recorderBegin();
  encodersBegin();

  if (imuBegin()) {
    Serial.printf("# calibrating gyro (%u ms) - keep the chair STILL\n", (unsigned)cfg::IMU_CAL_MS);
    if (!imuWaitReady(15000, uiUpdateLed))
      Serial.println("# gyro not calibrated yet (chair moving?) - heading from the encoders until it is");
  }

  Serial.println("# Short press / 'p' = start/pause recording, long press / 'd' = replay, 'w' = delete all,");
  Serial.println("# 'r' = reset position, 'g' = calibrate gyro, 'h' = heading source gyro/encoders");

  xTaskCreatePinnedToCore(odometryTask, "odometry", 4096, NULL, 3, NULL, 1);
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': recorderToggle(); break;
      case 'd': recorderReplayNext(); break;
      case 'w': recorderWipe(); break;
      case 'r': resetReq = true; Serial.println("# position reset"); break;
      case 'g': imuRecalibrate(); break;
      case 'h':
        gyroPreferred = !gyroPreferred;
        Serial.printf("# heading source: %s\n", gyroPreferred ? "gyro (encoders if the IMU is lost)" : "encoders only");
        break;
      default: break;
    }
  }
  uiUpdate();
  vTaskDelay(pdMS_TO_TICKS(20));
}
