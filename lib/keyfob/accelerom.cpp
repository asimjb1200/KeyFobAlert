#include <Arduino.h>
#include <Wire.h>
#include <accelerom.h>

bool verifyAccelConnection() {
  Wire.beginTransmission(ACCELEROMETER_ADDR);
  Wire.write(WHO_AM_I_REGISTER_ACCEL);
  uint8_t status = Wire.endTransmission(false);

  if (status == 0) {
    Serial.println("Slave sent an ACK");
    Wire.requestFrom(ACCELEROMETER_ADDR, 1);
    if (Wire.available()) {
      uint8_t data = Wire.read();
      Serial.print("WHO_AM_I: 0x");
      Serial.println(data, HEX);
      return true;
    }
  } else if (status == 2) {
    Serial.println("Received NACK on transmit of address");
  } else if (status == 3) {
    Serial.println("Received NACK on transmit of data");
  } else {
    Serial.println("4: Line busy, etc.");
  }
  Serial.flush();
  return false;
}

uint8_t sendDataToRegister(uint8_t deviceAddress, uint8_t deviceRegister, uint8_t command) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceRegister);
  Wire.write(command);
  return Wire.endTransmission();
}

bool initFreeFallDetection() {
  uint8_t status = 0;

  // turn on the sensor, enable X,Y, and Z. ODR = 100Hz
  uint8_t ctrl_reg1 = 0x20;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg1, 0x57);
  if (status != 0) { Serial.print("CTRL_REG1 failed, status: "); Serial.println(status); return false; }

  // High-pass filter disabled
  uint8_t ctrl_reg2 = 0x21;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg2, 0x00);
  if (status != 0) { Serial.print("CTRL_REG2 failed, status: "); Serial.println(status); return false; }

  // Interrupt activity 1 driven to INT1 pin
  uint8_t ctrl_reg3 = 0x22;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg3, 0x40);
  if (status != 0) { Serial.print("CTRL_REG3 failed, status: "); Serial.println(status); return false; }

  // FS = ±2 G
  uint8_t ctrl_reg4 = 0x23;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg4, 0x00);
  if (status != 0) { Serial.print("CTRL_REG4 failed, status: "); Serial.println(status); return false; }

  // interrupt 1 pin latched
  uint8_t ctrl_reg5 = 0x24;
  status = sendDataToRegister(ACCELEROMETER_ADDR, ctrl_reg5, 0x08);
  if (status != 0) { Serial.print("CTRL_REG5 failed, status: "); Serial.println(status); return false; }

  // set free-fall threshold = 350mg
  uint8_t int1_ths_reg = 0x32;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_ths_reg, 0x16);
  if (status != 0) { Serial.print("INT1_THS failed, status: "); Serial.println(status); return false; }

  // set minimum event duration
  uint8_t int1_duration_reg = 0x33;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_duration_reg, 0x03);
  if (status != 0) { Serial.print("INT1_DURATION failed, status: "); Serial.println(status); return false; }

  // configure free-fall recognition
  uint8_t int1_cfg_reg = 0x30;
  status = sendDataToRegister(ACCELEROMETER_ADDR, int1_cfg_reg, 0x95);
  if (status != 0) { Serial.print("INT1_CFG failed, status: "); Serial.println(status); return false; }

  // Serial.println("Free fall detection initialized successfully");
  //Serial.flush();
  return true;
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t deviceRegister) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(deviceRegister);
    Wire.endTransmission(false); // repeated start
    Wire.requestFrom(deviceAddress, 1);
    return Wire.read();
}

void initAccelInterruptPin() {
  // set pin 2 as input, which is PA4
  PORTA.DIRCLR = PIN4_bm;

  PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_LEVEL_gc;
}