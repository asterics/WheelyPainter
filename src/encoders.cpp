#include "encoders.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include "config.h"

namespace {

struct Quad {
  uint8_t pinA, pinB;
  volatile int32_t count;
  volatile uint8_t state;
};
Quad encL = {cfg::ENC_L_A, cfg::ENC_L_B, 0, 0};
Quad encR = {cfg::ENC_R_A, cfg::ENC_R_B, 0, 0};

// index = (previous state << 2) | new state, state = (A << 1) | B
const DRAM_ATTR int8_t QUAD_TABLE[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR quadISR(void *arg) {
  Quad *q = (Quad *)arg;
  uint8_t s = (uint8_t)((gpio_get_level((gpio_num_t)q->pinA) << 1) | gpio_get_level((gpio_num_t)q->pinB));
  q->count += QUAD_TABLE[(q->state << 2) | s];
  q->state = s;
}

void attachQuad(Quad &q) {
  pinMode(q.pinA, INPUT);  // note: use external pullups to 3V3!
  pinMode(q.pinB, INPUT);
  q.state = (uint8_t)((digitalRead(q.pinA) << 1) | digitalRead(q.pinB));
  attachInterruptArg(q.pinA, quadISR, &q, CHANGE);
  attachInterruptArg(q.pinB, quadISR, &q, CHANGE);
}

}  // namespace

void encodersBegin() {
  attachQuad(encL);
  attachQuad(encR);
  Serial.println("# wheel encoders enabled");
}

void encodersReadDeltas(float &dL, float &dR) {
  static int32_t lastL = 0, lastR = 0;
  int32_t cl = encL.count, cr = encR.count;
  dL = cfg::ENC_LEFT_SIGN * (float)(cl - lastL) * cfg::MM_PER_COUNT;
  dR = cfg::ENC_RIGHT_SIGN * (float)(cr - lastR) * cfg::MM_PER_COUNT * cfg::RIGHT_WHEEL_COMP;
  lastL = cl;
  lastR = cr;
}

int32_t encoderCountL() { return encL.count; }
int32_t encoderCountR() { return encR.count; }
