#pragma once
// GY-521 (MPU-6050) gyro module: provides the platform's yaw rotation, independent of wheel slip.
//
//  * runs its own task at IMU_HZ (200 Hz) and integrates the turn rate about the vertical axis
//  * "vertical" is taken from the accelerometer, so the board can be mounted in ANY orientation
//    and on a tilted chair / ramp the yaw rate is still measured correctly
//  * zero-rate offset (bias) is measured at start-up / on 'g' and re-learned whenever the chair
//    stands still
#include <stdint.h>

enum class ImuState : uint8_t { Off, Calibrating, Running };

// Initialise the sensor and start the sampling task. Returns false if the sensor does not answer
// (state stays Off; the firmware then falls back to encoder-only heading).
bool imuBegin();

ImuState imuState();
const char *imuStateName();
bool imuUsable();                          // running, calibrated and delivering data
bool imuWaitReady(uint32_t timeoutMs, void (*idle)());   // wait for the first calibration

// Start a new offset calibration (the chair must stand still for IMU_CAL_MS).
void imuRecalibrate();

// Tell the IMU task whether the wheels stand still (enables offset learning / drift freeze).
void imuSetWheelsStill(bool still);

// Heading change [rad, clockwise positive] since the previous call. Call from one task only.
float imuTakeYaw();

// Diagnostics for the status line
float imuYawRateDps();                     // filtered turn rate, clockwise positive
float imuBiasDps();                        // currently removed zero-rate offset about the vertical axis
float imuTempC();
