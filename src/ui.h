#pragma once
// Push button (short press = record, long press = replay) and status LED.
void uiBegin();
void uiUpdateLed();      // LED: fast blink while the gyro calibrates, slow blink while recording
void uiHandleButton();
inline void uiUpdate() { uiUpdateLed(); uiHandleButton(); }
