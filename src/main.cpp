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
volatile bool keepBmsAlive = false;

uint8_t debounceDelay = 150;
bool last_pressed;
uint32_t toggle_time;

// key pin for MH-CD42 battery mgmt chip
unsigned long pulseStartTime = 0;
bool isPulseActive = false;
const unsigned long KEY_PIN_PULSE_DURATION = 100; // 100ms pulse

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

void setupKeepBMSAlivePin() {
    // set pin PB3 to output
    PORTB.DIRSET = PIN3_bm;

    // pull it HIGH for now
    PORTB.OUT |= PIN3_bm;
}

/** pulse the key pin for around 100ms to keep the BMS from shutting down */
void keepBMSAlive() {

  if (millis() - pulseStartTime >= KEY_PIN_PULSE_DURATION) {
    // pull key pin back high
    PORTB.OUT |= PIN3_bm;
    pulseStartTime = 0;
    isPulseActive = false;
    keepBmsAlive = false;
  } 

  if (!isPulseActive) {
    // record the timestamp
    pulseStartTime = millis();
  }

  // pull the pin low
  if (!isPulseActive)
    PORTB.OUT &= ~PIN3_bm;
  
  isPulseActive = true;
}

/** Disable the RTC temporarily while the MCU is doing other things */
void disableRTC() {
  RTC.CTRLA &= ~RTC_RTCEN_bm;
}

void processMCUState() {
  bool pressed;
  // Serial.print("state=");
  // Serial.println((uint8_t)mcu_state);
  // Serial.flush();
  switch ((uint8_t)mcu_state)
  {
    case RESTING:
      if (keepBmsAlive) {
        keepBMSAlive();
      } else {
        deepSleepFlash();
        shutdownAmp();
        sleep_mode();
      }

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
      disableRTC();
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

      // re-enable the RTC
      RTC.CTRLA |= RTC_RTCEN_bm;

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

/**
 * You must always verify if your target number fits inside the hardware register box.
 * Target Ticks = Target Time x Clock Frequency
 * If Target Ticks > Register Max (65,535 for 16-bit, 255 for 8-bit), you must use a prescaler to slow down the clock frequency.
 */
void initRTC() {
  // wait for all registers to be synchronized
  while (RTC.STATUS > 0){}
  
  // use the internal 1kHz clock for the RTC
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;

  /**
   * Set the overflow value in the Period register.
   * the 16-bit RTC Counter uses a Period (PER) register that counts to any number up to 65,535 (clock ticks). 
   * To make this work for my limit of 20 seconds, I need a way to have 20 seconds represented in 65k clock ticks or less.
   * Since my chosen clock ticks at 1,024 a second (1,024Hz clock), I can calculate how many ticks occur in 20 seconds
   * 20 seconds x 1024 ticks/sec = 20,480 ticks. 20,480 fits cleanly inside a 16-bit register with plenty of room to spare
  */
  RTC.PER = 20480;

  /**
   * Enable the desired interrupts by writing to the respective interrupt enable bits (CMP, OVF). 
   * The moment RTC.CNT equals RTC.PER, the hardware triggers an Overflow Interrupt
  */
  RTC.INTCTRL = RTC_OVF_bm;

  RTC.DBGCTRL |= RTC_DBGRUN_bm;

  // Configure the RTC internal prescaler and Enable the RTC by writing a ‘1’ to the RTC Peripheral Enable bit
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
}

void setup() {
  initMCUClock();
  initRTC();
  setupKeepBMSAlivePin();

  //Serial.begin(115200);

  //delay(10000);

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

  uint8_t accelSetUp = initFreeFallDetection();

  delay(100);

  if (accelSetUp) {
    //select which sleep mode to enter and enable the sleep controller
  set_sleep_mode(SLEEP_MODE_STANDBY);
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

ISR(RTC_CNT_vect)
{
  RTC.INTFLAGS = RTC_OVF_bm;
  
  keepBmsAlive = true;
}