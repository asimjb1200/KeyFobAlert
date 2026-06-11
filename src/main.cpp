#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <mcu_state.h>
#include <accelerom.h>
#include <audio.h>
#include <flash.h>
#include <SPI.h>

volatile MCU_State_t mcu_state = RESTING;

void scanBusForDevices() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at: 0x");
      Serial.println(addr, HEX);
    }
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
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLB = 0;

  // selects the source for the Main Clock (internal crystal)
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc;

  // give time for clock to switch if necessary
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm){}
}

void setup() {
  initMCUClock();

  Serial.begin(115200);

  delay(10000);

  delay(100);
  Serial.println("int pin start");
  Serial.flush();
  initAccelInterruptPin();
  Serial.println("int pin done");
  Serial.flush();
  delay(100);

  Serial.println("stop audio start");
  Serial.flush();
  setupStopAudioPin();
  Serial.println("stop audio end");
  Serial.flush();

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
  delay(100);

  Wire.begin();
  delay(100);
  
  //delay(10000);
  
  //checkMCUAndAccelConnections();

  // initialize the CS pin for usage with SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  delay(100);
  // checkFlashConnection();
  // getFlashElectronicInfo();

  // start audio data from the beginning
  lastMemoryAddress = 0;
  Serial.println("buff fill start");
  Serial.flush();
  fillBuffer(); // get audio data ready
  Serial.println("buff fill end");
  Serial.flush();
  
  Serial.println("free fall init started");
  Serial.flush();
  uint8_t accelSetUp = initFreeFallDetection();
  Serial.println("Free fall detection initialized successfully");
  Serial.flush();
  delay(100);

  if (accelSetUp) {
    sei();

    // select which sleep mode to enter and enable the sleep controller
    set_sleep_mode(SLEEP_MODE_STANDBY);
  } else {
     Serial.println("set up failed");
  }
}

void loop()
{
  Serial.println("Sleep Mode");
  Serial.flush();

  if (mcu_state == MCU_State_t::RESTING)
  {
    sleep_mode();
  }

  // small delay to allow the full fall to take place
  //delay(2000);

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
      //readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);

      //wakeUpFlash();

      // enable the periodic timer for audio playback
      //enableHardwareTimer();
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
      //initAccelInterruptPin();

      // stop the periodic timer
      //disableHardwareTimer();

      // put the flash memory to sleep
      //deepSleepFlash();

      mcu_state = RESTING;
      break;

    default:
      break;
  }
}

ISR(PORTA_PORT_vect) {
    // Check to see if the interrupt came from the audio sht down pin
    if (PORTA.INTFLAGS & PIN5_bm) {
        PORTA.INTFLAGS &= PIN5_bm;

        mcu_state = DEVICE_RECOVERED;
        return;
    }

    // interrupt came from accelerometer
    if (PORTA.INTFLAGS & PIN4_bm) {
        PORTA.INTFLAGS &= PIN4_bm;
        // Disable the interrupt
        // PORTA.PIN4CTRL &= ~PORT_ISC_gm;
        // PORTA.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;

        // works better when I call it here..
        //readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);

        mcu_state = FALL_DETECTED;
        return;
    }
}