#pragma once
#include <stdint.h>

// Quadrature decoding of the two wheel encoders (GPIO interrupts, 4x).
void encodersBegin();

// Distance travelled by each wheel since the previous call [mm] (forward = positive).
// Call from one task only.
void encodersReadDeltas(float &dL, float &dR);

// Raw accumulated counts (for the status line).
int32_t encoderCountL();
int32_t encoderCountR();
