#include "ui.h"
#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "recorder.h"

void uiBegin() {
  pinMode(cfg::LED_PIN, OUTPUT);
  digitalWrite(cfg::LED_PIN, LOW);
  pinMode(cfg::BTN_PIN, INPUT_PULLUP);
}

// Button: short press (release before LONG_PRESS_MS) = start/pause, long press = replay next file
void uiHandleButton() {
  static bool down = false, longDone = false;
  static uint32_t t0 = 0;
  bool pressed = digitalRead(cfg::BTN_PIN) == LOW;
  uint32_t now = millis();

  if (pressed && !down) {
    down = true;
    longDone = false;
    t0 = now;
  } else if (pressed && down) {
    if (!longDone && now - t0 >= cfg::LONG_PRESS_MS) {
      longDone = true;
      recorderReplayNext();
    }
  } else if (!pressed && down) {
    down = false;
    if (!longDone && now - t0 >= cfg::DEBOUNCE_MS) recorderToggle();
  }
}

void uiUpdateLed() {
  static uint32_t last = 0;
  static bool on = false;
  uint32_t interval = 0;
  if (imuState() == ImuState::Calibrating) interval = cfg::CAL_BLINK_MS;
  else if (recorderIsRecording()) interval = cfg::BLINK_MS;

  if (interval == 0) {
    if (on) { digitalWrite(cfg::LED_PIN, LOW); on = false; }
    return;
  }
  uint32_t now = millis();
  if (now - last >= interval) {
    last = now;
    on = !on;
    digitalWrite(cfg::LED_PIN, on ? HIGH : LOW);
  }
}
