#pragma once
#include <stdint.h>
#define AUDIO_BTN_PRESSED (!(PORTA.IN & PIN5_bm))

void setupStopAudioPin();
void setupShutdownBuzzerPin();
void shutdownBuzzer();
void enableBuzzer();