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

void processMCUState() {
  Serial.print("state=");
  Serial.println((uint8_t)mcu_state);
  Serial.flush();
  switch ((uint8_t)mcu_state)
  {
    case RESTING:
      Serial.println("Resting State");
      Serial.flush();
      deepSleepFlash();
      sleep_mode();
      break;

     case FALL_DETECTED:
  //     Serial.println("Fall Detected");
  //     Serial.flush();
      // clear the interrupt & play audio
      // noInterrupts();
      // readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);
      // sei();
      //enableHardwareTimer();
      wakeUpFlash();
      //Serial.println("Flash ready");Serial.flush();
      mcu_state = AUDIO_PLAYING;
      break;
    
    case AUDIO_PLAYING:
      if (bufferOneNeedsFill || bufferTwoNeedsFill)
      {
        //noInterrupts();
        fillBuffer();
        //sei();
      }

      bool pressed = AUDIO_BTN_PRESSED;

      // Check if stop audio button was pressed (PA5 Pulled to GND)
      if (pressed && last_pressed == false && millis() - toggle_time > 100) {
        mcu_state = DEVICE_RECOVERED;
        Serial.print("recovered. Pressed=");Serial.println((int)pressed);Serial.flush();
      }
      if (pressed != last_pressed) {
        toggle_time = millis();
      }
      last_pressed = pressed;

      break;
    
    case DEVICE_RECOVERED:
      // fill up buffers for next round
      Serial.println("Recovering...");Serial.flush();
      //fillBuffer();
      

      // re-enable the fall interrupt
      //PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;

      mcu_state = RESTING;
      Serial.println("going to sleep");Serial.flush();
      break;

    case CHARGING:
      Serial.println("charging state!");
      break;

    default:
      Serial.println("Unknown state!");
      mcu_state = RESTING;
      break;
  }
}

void checkMCUAndAccelConnections() {
  uint8_t deviceID = SIGROW_DEVICEID0;
  uint8_t serialNum = SIGROW_SERNUM0;
  
  Serial.print("Device ID: 0x"); Serial.println(deviceID, HEX);

  Serial.print("Serial Num: 0x"); Serial.println(serialNum, HEX);

  Serial.flush();

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

  Serial.begin(115200);

  delay(3000);

  // initialize the CS pin for usage with SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  delay(100);

  // start audio data from the beginning
  lastMemoryAddress = 0;
  Serial.println("buff fill start");
  Serial.flush();
  fillBuffer(); // get audio data ready
  Serial.print("buff fill end. last memory address: ");Serial.println(lastMemoryAddress);
  Serial.flush();

  delay(100);
  Serial.println("int pin start");
  Serial.flush();
  initAccelInterruptPin();
  Serial.println("int pin done");
  Serial.flush();
  delay(100);

  Serial.println("DAC init begin");
  Serial.flush();
  setupDAC();
  Serial.println("DAC init complete");
  Serial.flush();
  delay(100);
  
  Serial.println("Timer set up begin");
  Serial.flush();
  initHardwareTimer();
  Serial.println("Timer set up complete");
  Serial.flush();

  Serial.println("stop audio start");
  Serial.flush();
  setupStopAudioPin();
  Serial.println("stop audio end");
  Serial.flush();
  
  Wire.begin();
  delay(100);

  
  //checkMCUAndAccelConnections();

  // checkFlashConnection();
  // getFlashElectronicInfo();
  
  Serial.println("free fall init started");
  Serial.flush();
  uint8_t accelSetUp = initFreeFallDetection();
  Serial.println("Free fall detection initialized successfully");
  Serial.flush();
  delay(100);

  if (accelSetUp) {
    //select which sleep mode to enter and enable the sleep controller
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  } else {
     Serial.println("set up failed");
  }
}

void loop()
{
  int currentState = (int)mcu_state; 

  Serial.print("state=");
  Serial.println((int)currentState);
  Serial.flush();

  if (currentState == MCU_State_t::RESTING) {
      // RESTING
      Serial.println("Resting State");
      Serial.flush();
      deepSleepFlash();
      sleep_mode();
  } else if (currentState == MCU_State_t::FALL_DETECTED) {
    // FALL_DETECTED
    wakeUpFlash();
    //Serial.println("Flash ready");Serial.flush();
    mcu_state = AUDIO_PLAYING;
  } else if (currentState == MCU_State_t::AUDIO_PLAYING) {
    // AUDIO_PLAYING
    if (bufferOneNeedsFill || bufferTwoNeedsFill)
    {
      //noInterrupts();
      fillBuffer();
      //sei();
    }

    bool pressed = AUDIO_BTN_PRESSED;

    // Check if stop audio button was pressed (PA5 Pulled to GND)
    if (pressed && last_pressed == false && millis() - toggle_time > 100) {
      mcu_state = DEVICE_RECOVERED;
      Serial.print("recovered. Pressed=");Serial.println((int)pressed);Serial.flush();
    }
    if (pressed != last_pressed) {
      toggle_time = millis();
    }
    last_pressed = pressed;
  } else if (currentState == MCU_State_t::DEVICE_RECOVERED) {
      Serial.println("Force-entering Recovered Block!");
      Serial.flush();
      // RE-enable accelerator interrupt pin
      PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;
      mcu_state = RESTING;
  } else {
      Serial.print("Unknown state value: ");
      Serial.println(currentState);
  }
}

ISR(PORTA_PORT_vect) {
  if (INTRPT_FROM_ACCELR) {
    PORTA.INTFLAGS &= PIN4_bm;

    // Disable the interrupt
    disableAccelInterruptPin();

    mcu_state = FALL_DETECTED;
    return;
  }
}