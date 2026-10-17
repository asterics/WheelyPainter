#pragma once
// All tunable settings of the firmware in one place.
#include <Arduino.h>

namespace cfg {

// ------------------------------------------------------------------ timing
constexpr int SAMPLE_HZ = 20;                     // odometry / output rate

// ------------------------------------------------------------------ mechanics (MEASURE YOURS)
// The encoders are pressed against the tyres, so the distance per count is set by the diameter
// of the small measuring roller on the encoder shaft (not by the wheelchair wheel).
constexpr float WHEEL_DIAMETER_MM = 56.5f;        // diameter of the encoder roller
constexpr float TRACK_WIDTH_MM = 560.0f;          // distance between the two wheelchair wheel contact points

// ------------------------------------------------------------------ wheel encoders (quadrature, 4x decoding)
constexpr int ENCODER_PPR = 600;                  // pulses per revolution per channel
constexpr int ENC_L_A = 25, ENC_L_B = 26;
constexpr int ENC_R_A = 14, ENC_R_B = 27;
constexpr int ENC_LEFT_SIGN = 1;                  // set to -1 if a wheel counts backwards when driving forward
constexpr int ENC_RIGHT_SIGN = 1;                 // (the right encoder is usually mirrored -> -1)
constexpr float MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / (4.0f * ENCODER_PPR);
// Right wheel distance is multiplied by this (see README, "Calibration"). 1.0 = no compensation.
constexpr float RIGHT_WHEEL_COMP = 1.020f;

// ------------------------------------------------------------------ IMU (GY-521 / MPU-6050 on I2C)
constexpr int IMU_SDA = 21, IMU_SCL = 22;
constexpr uint32_t IMU_I2C_HZ = 400000;
constexpr uint8_t IMU_ADDR = 0x68;                // 0x68 = AD0 to GND, 0x69 = AD0 to 3V3
constexpr int IMU_HZ = 200;                       // gyro sampling / integration rate
constexpr bool IMU_INVERT_YAW = false;            // set to true if a clockwise turn gives a negative heading change
constexpr float IMU_YAW_SCALE = 1.0f;              // gyro scale correction, see README "Calibration" (MPU-6050: +-3 % from factory)

// Start-up / 'g' offset calibration: average the gyro while the chair stands still.
constexpr uint32_t IMU_CAL_MS = 2000;             // length of the averaging window
constexpr float IMU_CAL_MAX_SPREAD_DPS = 2.0f;    // max (peak-to-peak) gyro noise allowed in the window
constexpr float IMU_CAL_MAX_ACC_SPREAD_G = 0.05f; // max peak-to-peak variation of |accel| in the window

// Running filters
constexpr float IMU_GRAVITY_TAU_S = 1.0f;         // time constant of the "which way is up" low-pass
constexpr float IMU_BIAS_TAU_S = 3.0f;            // time constant of the offset re-learning while standing still
constexpr float IMU_BIAS_GATE_DPS = 3.0f;         // only learn the offset while |rotation| is below this
constexpr float IMU_FREEZE_DPS = 0.5f;            // while standing still and below this, heading is frozen

// Standstill detection (from the wheel encoders)
constexpr uint32_t STILL_MS = 300;                // no wheel motion for this long = standing still
constexpr float STILL_WHEEL_MM = 0.25f;           // wheel travel per sample below this counts as "no motion"

// Slip detection: encoder turn rate and gyro turn rate disagree by more than
// max(SLIP_MIN_DPS, SLIP_REL * |gyro rate|) while driving
constexpr float SLIP_MIN_DPS = 5.0f;
constexpr float SLIP_REL = 0.10f;

// ------------------------------------------------------------------ button and LED
constexpr int BTN_PIN = 13;                       // push button between GPIO13 and GND (internal pull-up)
constexpr int LED_PIN = 2;                        // internal LED of the ESP32 DevKit
constexpr uint32_t DEBOUNCE_MS = 40;
constexpr uint32_t LONG_PRESS_MS = 1000;          // press longer than this = replay
constexpr uint32_t BLINK_MS = 250;                // LED toggles every 250 ms while recording
constexpr uint32_t CAL_BLINK_MS = 100;            // fast blinking while the gyro is being calibrated

// ------------------------------------------------------------------ recording
constexpr size_t LOG_BUF = 4096;
constexpr uint32_t LOG_FLUSH_MS = 2000;
constexpr size_t LOG_MAX_BYTES = 1000000;         // one recording stops at ~1 MB
constexpr size_t MIN_FREE_BYTES = 65536;          // do not start a recording with less free flash

}  // namespace cfg
