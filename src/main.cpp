#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <audio.h>
#include <flash.h>
#include <accelerom.h>

#define DAC_PIN PIN_PA6

void setup()
{
  sei(); // Enable global interrupts. all interrupts are 0 level by default on this chip
  bool flashSetup = setupFlash();
  setupAudioInterruptTimer();
  accelInterruptSetup();
  accelRegisterConfig();
  // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  // 8MHz clock, MSB first, Mode 0 (CPOL=0, CPHA=0)
}

void loop()
{
  // check for interrupts from accelerometer
  accelCheckForInterruptEvents();

  if (free_fall_detected && impact_detected && !audio_playing)
  {
      // play audio
      enableAudio();
  }

  if (request_audio_stop) // button was pressed
  {
      pauseAudio();
      deviceRecovered();
  }

  checkAndFillEmptyAudioBuffer();
}