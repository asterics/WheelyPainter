#include "odometry.h"
#include <math.h>
#include "config.h"

namespace {
constexpr float PI_F = 3.14159265358979f;
constexpr uint16_t STILL_SAMPLES = (cfg::STILL_MS * cfg::SAMPLE_HZ / 1000) > 0 ? (cfg::STILL_MS * cfg::SAMPLE_HZ / 1000) : 1;
}

void Odometry::reset() {
  x_ = y_ = th_ = v_ = 0;
  thEncTotal_ = thGyroTotal_ = 0;
  slipSamples_ = 0;
  stillCount_ = 0;
  still_ = false;
  usingGyro_ = false;
}

float Odometry::headingDeg() const { return th_ * RAD2DEG; }

Odometry::Step Odometry::update(float dL, float dR, float dthGyro, bool gyroUsable, float dt) {
  const float ds = 0.5f * (dL + dR);                          // distance travelled by the platform centre
  const float dthEnc = (dL - dR) / cfg::TRACK_WIDTH_MM;       // left faster than right -> turn clockwise

  // standstill: no wheel motion for STILL_MS
  if (fabsf(dL) < cfg::STILL_WHEEL_MM && fabsf(dR) < cfg::STILL_WHEEL_MM) {
    if (stillCount_ < 0xFFFF) stillCount_++;
  } else {
    stillCount_ = 0;
  }
  still_ = stillCount_ >= STILL_SAMPLES;

  // slip: while driving, the wheels and the gyro must agree on how fast the chair turns
  bool slip = false;
  if (gyroUsable && !still_ && dt > 0) {
    float rg = dthGyro / dt * RAD2DEG, re = dthEnc / dt * RAD2DEG;
    float limit = fmaxf(cfg::SLIP_MIN_DPS, cfg::SLIP_REL * fabsf(rg));
    slip = fabsf(rg - re) > limit;
    if (slip) slipSamples_++;
  }

  usingGyro_ = gyroUsable && preferGyro_;
  const float dth = usingGyro_ ? dthGyro : dthEnc;

  const float mid = th_ + 0.5f * dth;                         // midpoint integration
  Step st;
  st.dx = ds * sinf(mid);
  st.dy = ds * cosf(mid);
  st.slip = slip;
  x_ += st.dx;
  y_ += st.dy;
  th_ += dth;
  if (th_ > PI_F) th_ -= 2 * PI_F;
  if (th_ < -PI_F) th_ += 2 * PI_F;

  thEncTotal_ += dthEnc;
  if (gyroUsable) thGyroTotal_ += dthGyro;
  if (dt > 0) v_ = ds / dt;
  return st;
}
