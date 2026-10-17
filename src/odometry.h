#pragma once
#include <stdint.h>

// Differential-drive odometry with gyro/encoder heading fusion. Pure computation (no hardware access),
// called once per sample period.
//
//   distance : from the wheel encoders      ds  = (dL + dR) / 2
//   heading  : from the gyro (if available) dθ  = gyro yaw change;  otherwise from the encoders
//                                           dθ  = (dL - dR) / track width
//   position : x += ds·sin(θ + dθ/2),  y += ds·cos(θ + dθ/2)
//
// Heading θ is measured clockwise from +Y ("up" in the viewer), in radians internally.
class Odometry {
 public:
  struct Step {
    float dx, dy;       // displacement of this step [mm]
    bool slip;          // encoder and gyro turn rates disagree
  };

  Odometry() { reset(); }

  // Position and heading back to 0 (heading 0 = +Y again).
  void reset();

  // dL, dR: wheel travel [mm]; dthGyro: gyro heading change [rad, clockwise]; gyroUsable: IMU delivers data;
  // dt: seconds since the last call.
  Step update(float dL, float dR, float dthGyro, bool gyroUsable, float dt);

  // true: use the gyro for the heading whenever it is usable; false: encoders only (for comparison)
  void setPreferGyro(bool b) { preferGyro_ = b; }

  float x() const { return x_; }
  float y() const { return y_; }
  float speed() const { return v_; }                  // [mm/s]
  float headingDeg() const;                           // used heading, -180..180
  float encHeadingDeg() const { return thEncTotal_ * RAD2DEG; }    // encoders only, continuous since reset
  float gyroHeadingDeg() const { return thGyroTotal_ * RAD2DEG; }  // gyro only, continuous since reset
  bool usingGyro() const { return usingGyro_; }
  bool wheelsStill() const { return still_; }
  uint32_t slipSamples() const { return slipSamples_; }

 private:
  static constexpr float RAD2DEG = 57.29577951308232f;
  float x_, y_, th_, v_;
  float thEncTotal_, thGyroTotal_;
  uint32_t slipSamples_;
  uint16_t stillCount_;
  bool still_, usingGyro_, preferGyro_ = true;
};
