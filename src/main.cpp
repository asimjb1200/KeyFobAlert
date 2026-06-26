#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <mcu_state.h>
#include <accelerom.h>
#include <audio.h>
#include <flash.h>
#include <SPI.h>
#include <avr/io.h>
#include <util/delay.h>
extern "C" {
    #include <avr/cpufunc.h>
}


volatile MCU_State_t mcu_state = RESTING;

uint8_t debounceDelay = 150;
bool last_pressed;
uint32_t toggle_time;

void scanBusForDevices() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at: 0x");
      Serial.println(addr, HEX);
    }
  }
}

void updateMCUState(MCU_State_t desiredState) {
  mcu_state = desiredState;
}

void processMCUState() {
  bool pressed;
  // Serial.print("state=");
  // Serial.println((uint8_t)mcu_state);
  // Serial.flush();
  switch ((uint8_t)mcu_state)
  {
    case RESTING:
      deepSleepFlash();
      shutdownAmp();
      sleep_mode();
      break;

     case FALL_DETECTED:
  //     Serial.println("Fall Detected");
  //     Serial.flush();
      // clear the interrupt & play audio
      // noInterrupts();
      // readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);
      // sei();

      enableAmp();
      wakeUpFlash();
      updateMCUState(AUDIO_PLAYING);
      break;
    
    case AUDIO_PLAYING:
      fillBuffer();

      pressed = AUDIO_BTN_PRESSED;

      // Check if stop audio button was pressed (PA5 Pulled to GND)
      if (pressed && last_pressed == false && millis() - toggle_time > 100) {
        updateMCUState(DEVICE_RECOVERED);
      }

      if (pressed != last_pressed) {
        toggle_time = millis();
      }
      last_pressed = pressed;

      break;
    
    case DEVICE_RECOVERED:
      // fill up buffers for next round
      fillBuffer();
      

      // re-enable the fall interrupt
      enableAccelInterruptPin();

      updateMCUState(RESTING);
      break;

    case CHARGING:
      //Serial.println("charging state!");
      break;

    default:
      //Serial.println("Unknown state!");
      updateMCUState(RESTING);
      break;
  }
}

void checkMCUAndAccelConnections() {
  uint8_t deviceID = SIGROW_DEVICEID0;
  uint8_t serialNum = SIGROW_SERNUM0;
  
  // Serial.print("Device ID: 0x"); Serial.println(deviceID, HEX);

  // Serial.print("Serial Num: 0x"); Serial.println(serialNum, HEX);

  // Serial.flush();

  scanBusForDevices();

  uint8_t okayToSetup = verifyAccelConnection();
}

void initMCUClock() 
{
  // disable prescaler
  ccp_write_io((uint8_t*)&CLKCTRL.MCLKCTRLB, 0x00);

  // selects the source for the Main Clock (internal crystal)
  ccp_write_io((uint8_t*)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);
  

  // give time for clock to switch if necessary
  while (!(CLKCTRL.MCLKSTATUS & CLKCTRL_OSC20MS_bm)) {
        // Do nothing until the 20MHz oscillator is fully stable
  }
}

void setup() {
  initMCUClock();

  //Serial.begin(115200);

  // initialize the CS pin for usage with SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  delay(100);

  // start audio data from the beginning
  lastMemoryAddress = 0;

  fillBuffer(); // get audio data ready

  initAccelInterruptPin();

  setupDAC();

  setupShutdownAmpPin();

  initHardwareTimer();

  setupStopAudioPin();
  
  Wire.begin();
  delay(100);

  
  //checkMCUAndAccelConnections();

  // checkFlashConnection();
  // getFlashElectronicInfo();
  
  // Serial.println("free fall init started");
  // Serial.flush();
  uint8_t accelSetUp = initFreeFallDetection();
  // Serial.println("Free fall detection initialized successfully");
  // Serial.flush();
  delay(100);

  if (accelSetUp) {
    //select which sleep mode to enter and enable the sleep controller
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  } else {
     //Serial.println("set up failed");
  }
}

void loop()
{
  processMCUState();
}

ISR(PORTA_PORT_vect) {
  if (INTRPT_FROM_ACCELR) {
    PORTA.INTFLAGS = PIN4_bm;

    // Disable the interrupt
    disableAccelInterruptPin();

    mcu_state = FALL_DETECTED;
    return;
  }
}