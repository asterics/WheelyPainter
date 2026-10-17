#pragma once
// One odometry sample and its CSV representation (serial stream and recording files).
#include <Arduino.h>

struct OdoSample {
  uint32_t t_ms;
  float dx, dy;     // displacement since last sample [mm]
  float x, y;       // accumulated position [mm]
  uint8_t n;        // number of valid wheels (always 2, kept for file compatibility)
  float h;          // heading [deg], 0 = +Y, clockwise positive, -180..180
};

static const char *const CSV_HEADER = "# recording; t_ms,dx_mm,dy_mm,x_mm,y_mm,n_valid,heading_deg\n";

static inline int fmtSample(char *out, size_t cap, const OdoSample &s) {
  return snprintf(out, cap, "%lu,%.3f,%.3f,%.2f,%.2f,%u,%.2f\n", (unsigned long)s.t_ms, s.dx, s.dy,
                  s.x, s.y, s.n, s.h);
}
