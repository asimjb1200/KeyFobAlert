#pragma once
#include <stdint.h>
#define AUDIO_BTN_PRESSED (!(PORTA.IN & PIN5_bm))

void setupShutdownBuzzerPin();
void shutdownBuzzer();
void enableBuzzer();