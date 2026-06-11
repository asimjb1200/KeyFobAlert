#pragma once
#include <stdint.h>
#define DAC PIN_PA6
#define STOP_AUDIO_PIN PIN_PA5

extern uint32_t lastMemoryAddress;
extern volatile uint16_t bytesSent;
extern volatile bool bufferOneNeedsFill;
extern volatile bool bufferTwoNeedsFill;
extern volatile bool usingBufferOne;
extern uint8_t audioBufferOne[256];
extern uint8_t audioBufferTwo[256];

void setupDAC();
void enableHardwareTimer();
void disableHardwareTimer();
void initHardwareTimer();
void fillBuffer();
void stopAudio();
void setupStopAudioPin();