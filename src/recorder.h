#pragma once
// Live serial output, recording to flash (LittleFS), replay and delete.
#include "sample.h"

// Mount the file system and start the logger task.
bool recorderBegin();

// Called by the odometry task for every sample (queued for the logger task).
void recorderSubmit(const OdoSample &s);

bool recorderIsRecording();
bool recorderStreamEnabled();      // live serial output on?

void recorderToggle();             // start / pause recording (short press, 'p')
void recorderReplayNext();         // dump the next recording over serial (long press, 'd')
void recorderWipe();               // delete all recordings ('w')
