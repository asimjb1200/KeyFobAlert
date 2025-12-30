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
  uint8_t status;
  sei(); // Enable global interrupts. all interrupts are 0 level by default on this chip
  bool flashSetup = setupFlash();
  if (flashSetup == false)
  {
    while (1);
  }
  setupAudioInterruptTimer();
  accelInterruptPinSetup();
  status = accelRegisterConfig();
  if (status != AccelConfigError::ACCEL_SUCCESS) {
    Serial.print("Problem with accel config. Status code: ");
    Serial.println(status);
    while(1);
  }
  // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  // 8MHz clock, MSB first, Mode 0 (CPOL=0, CPHA=0)
}

void loop()
{
  // check for interrupts from accelerometer
  accelCheckForInterruptEvents();

  if (free_fall_detected && !audio_playing)
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