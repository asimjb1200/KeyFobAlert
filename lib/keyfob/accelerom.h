#pragma once
#include <stdint.h>
#include <mcu_state.h>

#define ACCELEROMETER_ADDR 0x18
#define SCL_PIN PIN_PB0 // PIN 9
#define SDA_PIN PIN_PB1 // PIN 8
#define WHO_AM_I_REGISTER_ACCEL 0x0F
#define INT1_SRC_REGISTER 0x31
#define INTERRUPT_PIN_CTRL PORTA.PIN1CTRL
#define INTRPT_FROM_ACCELR ((PORTA.INTFLAGS & PIN4_bm))

uint8_t sendDataToRegister(uint8_t deviceAddress, uint8_t deviceRegister, uint8_t command);
bool initFreeFallDetection();
void initAccelInterruptPin();
bool verifyAccelConnection();
uint8_t readRegister(uint8_t deviceAddress, uint8_t deviceRegister);
void disableAccelInterruptPin();
void enableAccelInterruptPin();