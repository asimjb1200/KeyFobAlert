#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <mcu_state.h>
#include <accelerom.h>
#include <audio.h>
#include <flash.h>
#include <SPI.h> 

volatile MCU_State_t mcu_state = RESTING;

void initMCUClock() 
{
  // disable prescaler
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0 << CLKCTRL_PEN_bp;

  // Set the clock to use 20MHz
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;

  // give time for clock to switch if necessary
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm){}
}

void setup() {
  initMCUClock();

  Serial.begin(115200);
  // Initialize SPI hardware (Pins 11, 12, 13)
  SPI.begin();
  Wire.begin();

  initAccelInterruptPin();
  initFreeFallDetection();
  setupDAC();
  initHardwareTimer();

  fillBuffer(); // get audio data ready

    // Setup Chip Select
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Ensure flash is 'OFF' (Active Low)

  // start the audio data from the beginning
  lastMemoryAddress = 0;

  sei();
  
  delay(10000);
  
  // uint8_t deviceID = SIGROW_DEVICEID0;
  // uint8_t serialNum = SIGROW_SERNUM0;
  
  // Serial.print("Device ID: 0x"); Serial.println(deviceID, HEX);

  // Serial.print("Serial Num: 0x"); Serial.println(serialNum, HEX);

  // Serial.flush();

  

  //uint8_t okayToSetup = verifyAccelConnection();

  // if (okayToSetup) {
  //   uint8_t accelIntSuccess = initFreeFallDetection();
  // }
  
  // select which sleep mode to enter and enable the sleep controller
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop()
{
  Serial.println("Sleep Mode");
  Serial.flush();

  if (mcu_state != MCU_State_t::AUDIO_PLAYING)
  {
    sleep_mode();
  }

  // small delay to allow the full fall to take place
  delay(2000);

  switch (mcu_state)
  {
    case RESTING:
      Serial.println("Resting State");
      Serial.flush();
      break;

    case FALL_DETECTED:
      Serial.println("Fall Detected");
      Serial.flush();
      // clear the interrupt & play audio
      readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);

      wakeUpFlash();

      // enable the periodic timer for audio playback
      enableHardwareTimer();
      mcu_state = AUDIO_PLAYING;
      break;
    
    case AUDIO_PLAYING:
      if (bufferOneNeedsFill || bufferTwoNeedsFill)
      {
        fillBuffer();
      }

      break;
    
    case DEVICE_RECOVERED:
      // fill up buffers for next round
      fillBuffer();

      // re-enable the fall interrupt
      initAccelInterruptPin();

      // stop the periodic timer
      disableHardwareTimer();

      // put the flash memory to sleep
      deepSleepFlash();

      mcu_state = RESTING;
      break;

    default:
      break;
  }
}