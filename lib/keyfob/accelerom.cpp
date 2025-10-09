#include <Wire.h>
#include <accelerom.h>

#define ACCEL_INT_PIN PIN_PB3

#define ACCEL_ADDRESS 0x18
#define ACCEL_CTRL_REG1 0X20
#define ACCEL_CTRL_REG2 0X21
#define ACCEL_CTRL_REG3 0X22
#define ACCEL_CTRL_REG4 0X23
#define ACCEL_CTRL_REG5 0X24

#define ACCEL_INT1_CFG 0x30
#define ACCEL_INT1_SRC 0x31
#define ACCEL_INT1_THS 0x32
#define ACCEL_INT1_DURATION 0X33

#define ACCEL_BUFFER_SIZE 0x03
#define ACCEL_SCL PIN_PB0
#define ACCEL_SDA PIN_PB1

#define OUT_X_L 0x28 // Memory address of the X-axis low byte
#define OUT_X_H 0x29 // Memory address of the X-axis high byte
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

volatile bool free_fall_detected = false;

static int readFromAccel(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z)
{   
    uint8_t status;

    Wire.beginTransmission(ACCEL_ADDRESS);

    // To enable address auto-increment, set the MSB of the register address to 1.
    // 0x28 becomes 0x28 | 0x80 = 0xA8
    Wire.write(OUT_X_L | 0x80); // Start reading at OUT_X_L and automatically increment
    status = Wire.endTransmission(false);
    if (status != 0) {

      return -1;
    }

    // Request 6 bytes of data. The accelerometer will send the bytes from OUT_X_L and OUT_X_H
    Wire.requestFrom(ACCEL_ADDRESS, 6);

    if (Wire.available() == 6)
    {
        // Read the two bytes from X
        uint8_t x_low = Wire.read();
        uint8_t x_high = Wire.read();
        // Combine them into a 16-bit signed integer
        // The LIS2DH12 uses little-endian format (low byte first)
        raw_x = (x_high << 8) | x_low;

        // two bytes from Y
        uint8_t y_low = Wire.read();
        uint8_t y_high = Wire.read();
        raw_y = (y_high << 8) | y_low;

        // two bytes from Z
        uint8_t z_low = Wire.read();
        uint8_t z_high = Wire.read();
        raw_z = (z_high << 8) | z_low;

        return 0;
    } else {
      return -1;
    }
}

// Determine the Event:
//  If the magnitude is low (e.g., less than 0.2g), likely a free-fall event.
//  If the magnitude is high (e.g., greater than 2g) impact event
static void determineEvent()
{
    int8_t status;
    int16_t raw_x, raw_y, raw_z;
    status = readFromAccel(raw_x, raw_y, raw_z);
    if (status != 0) {

    }

    // make sense of the readings to distinguish between free fall and impact based on g force

    // Right-Shift for 10-bit Data
    int16_t shifted_x = raw_x >> 6;
    int16_t shifted_y = raw_y >> 6;
    int16_t shifted_z = raw_z >> 6;

    float x_g = (float)shifted_x * 4.0 / 1000.0;
    float y_g = (float)shifted_y * 4.0 / 1000.0;
    float z_g = (float)shifted_z * 4.0 / 1000.0;

    float magnitude = sqrt(x_g * x_g + y_g * y_g + z_g * z_g);

    /**
     * In free fall, the sum of the squares of the acceleration components 
     * (x² + y² + z²) should be much less than 1 g²
     */
    if (magnitude < 0.3 && magnitude > 0) { 
        free_fall_detected = true;
    } else {
        free_fall_detected = false;
    }

    // Read the source register so that the pin can go back low
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_SRC);
    Wire.endTransmission(false);
    Wire.requestFrom(ACCEL_ADDRESS, 1);
}

void printI2CStatus(uint8_t status) {
  Serial.print("I2C Status Code: ");
  Serial.print(status);
  Serial.print(" - ");
  
  switch (status) {
    case 0:
      Serial.println("Success");
      break;
    case 1:
      Serial.println("Data too long for transmit buffer");
      break;
    case 2:
      Serial.println("NACK on address (device not found at 0x19)");
      break;
    case 3:
      Serial.println("NACK on data transmission");
      break;
    case 4:
      Serial.println("Other I2C error");
      break;
    case 5:
      Serial.println("Timeout");
      break;
    default:
      Serial.println("Unknown status code");
      break;
  }
}

void handleAccelError(int errorCode, uint8_t status) {
  switch (errorCode) {
    case ACCEL_SUCCESS:
      Serial.println("Accelerometer configured successfully");
      break;
      
    case ACCEL_CTRL_REG1_FAILED:
      Serial.println("ERROR: Failed to write CTRL_REG1 (data rate/axis enable)");
      printI2CStatus(status);
      Serial.println("Check: I2C connection, device power, pull-up resistors");
      break;

    case ACCEL_CTRL_REG2_FAILED:
      Serial.println("ERROR: Failed to write CTRL_REG2 (high pass filter)");
      printI2CStatus(status);
      Serial.println("Check: I2C connection, device power, pull-up resistors");
      break;
      
    case ACCEL_CTRL_REG4_FAILED:
      Serial.println("ERROR: Failed to write CTRL_REG4 (full scale/BDU)");
      printI2CStatus(status);
      Serial.println("Check: Device may be unresponsive or in incorrect mode");
      break;
      
    case ACCEL_CTRL_REG3_FAILED:
      Serial.println("ERROR: Failed to write CTRL_REG3 (interrupt routing)");
      printI2CStatus(status);
      Serial.println("Check: Device communication integrity");
      break;
      
    case ACCEL_INT1_THS_FAILED:
      Serial.println("ERROR: Failed to write INT1_THS (interrupt threshold)");
      printI2CStatus(status);
      Serial.println("Check: Register address correct, device responsive");
      break;
      
    case ACCEL_INT1_DURATION_FAILED:
      Serial.println("ERROR: Failed to write INT1_DURATION (interrupt duration)");
      printI2CStatus(status);
      Serial.println("Check: Previous register writes may have failed");
      break;
      
    case ACCEL_INT1_CFG_FAILED:
      Serial.println("ERROR: Failed to write INT1_CFG (interrupt configuration)");
      printI2CStatus(status);
      Serial.println("Check: Free fall detection setup may be incomplete");
      break;
      
    case ACCEL_CTRL_REG5_FAILED:
      Serial.println("ERROR: Failed to write CTRL_REG5 (interrupt latch)");
      printI2CStatus(status);
      Serial.println("Check: Final configuration step failed");
      break;
      
    default:
      Serial.print("ERROR: Unknown error code: ");
      Serial.println(errorCode);
      printI2CStatus(status);
      break;
  }
}

void accelInterruptPinSetup()
{
    // set pin as input
    PORTB.DIRCLR = PIN3_bm; // 0b00001000

    /*
    * When no interrupt is active, the pin is held low. 
    * When an interrupt occurs, the pin will be driven high
    * so use rising edge to detect interrupts
    */
    PORTB.PIN3CTRL = PORT_ISC_RISING_gc; // interrupt on rising edge
    PORTB.INTFLAGS = PIN3_bm;            // clear any stale flags
}

int accelRegisterConfig()
{
    uint8_t status;

    // set data rate to 100Hz and enable all axis 57h
    uint8_t dataRateConfig = 0b01010111;

    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG1);
    Wire.write(dataRateConfig);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_CTRL_REG1_FAILED, status);
        return -1;
    }

    // High-pass filter disabled 00h
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG2);
    Wire.write(0b00000000);
    status = Wire.endTransmission();
    if (status != 0) {
      handleAccelError(AccelConfigError::ACCEL_CTRL_REG2_FAILED, status);
      return -1;
    }

    // activate the interrupts on physical INT1 pin
    // INTERRUPT ACTIVE = IA
    uint8_t ctrl_reg3_cfg = 0b01000000; // I1_IA1 = 1
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG3);
    Wire.write(ctrl_reg3_cfg);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_CTRL_REG3_FAILED, status);
        return -1;
    }

    // block update, little endian, +/-2g resolution, normal operating mode
    uint8_t ctrlReg4Config = 0b10000000;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG4);
    Wire.write(ctrlReg4Config);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_CTRL_REG4_FAILED, status);
        return -1;
    }

    /**
    * With latching enabled, once the interrupt pin goes high,
    * it stays high until you read the source register (INT1_SRC at 0x31)
    */ 
    uint8_t ctrl_reg5_cfg = 0b00001000;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG5);
    Wire.write(ctrl_reg5_cfg);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_CTRL_REG5_FAILED, status);
        return -1;
    }

    /**
     * To detect free-fall, you need a low-g threshold
     * a good threshold is ≈ 100 mg so 7h is 112mg
     */
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_THS);
    Wire.write(0b00000111);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_INT1_THS_FAILED, status);
        return -1;
    }

    // set the duration needed before the interrupt event is considered valid
    // I want 50ms so since 1 LSB = 1/0DR ~ 10ms I can use 5 as the multiplier
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_DURATION);
    Wire.write(0b00000101);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_INT1_DURATION_FAILED, status);
        return -1;
    }

    // CONFIGURE INTERRUPT (INT1_CFG)
    // 0b10010101:
    //   Bit 7 (AOI)  = 1 (AND logic for all enabled events)
    //   Bit 4 (ZLIE) = 1 (Enable Z Low-g interrupt)
    //   Bit 2 (YLIE) = 1 (Enable Y Low-g interrupt)
    //   Bit 0 (XLIE) = 1 (Enable X Low-g interrupt)
    uint8_t int1_cfg_reg = 0b10010101;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_CFG);
    Wire.write(int1_cfg_reg);
    status = Wire.endTransmission();
    if (status != 0) {
        handleAccelError(AccelConfigError::ACCEL_INT1_CFG_FAILED, status);
        return -1;
    }

    handleAccelError(AccelConfigError::ACCEL_SUCCESS, status);
    return AccelConfigError::ACCEL_SUCCESS;
}

static void manualTwiSetup()
{
    PORTB.PIN0CTRL &= ~PORT_PULLUPEN_bm; // SCL
    PORTB.PIN1CTRL &= ~PORT_PULLUPEN_bm; // SDA

    // Set TWI0 (I2C) master baud rate for 100kHz
    // F_CLK_PER / F_SCL - 10 = 20,000,000 / 100,000 - 10 = 190
    TWI0.MBAUD = 190;

    TWI0.MCTRLA = TWI_ENABLE_bm | TWI_SMEN_bm;
    TWI0.CTRLA = TWI_SDASETUP_enum::TWI_SDASETUP_8CYC_gc;
    // 4. Set the bus to idle state to start
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

void accelCheckForInterruptEvents()
{
    if (free_fall_detected)
    {
        // process force data
        determineEvent();
    }
    else
    {
        return;
    }
}

void deviceRecovered() {
    free_fall_detected = false;
    //impact_detected = false;
}

// Define the Interrupt Service Routine for all pins on Port B
ISR(PORTB_PORT_vect)
{
    // Check if PB3 (my freefall pin) caused the interrupt
    if (PORTB.INTFLAGS & PIN3_bm) {
        // Clear the interrupt flag of the pin that caused the interrupt
        PORTB.INTFLAGS = PIN3_bm;

        // code to handle the accelerometer event goes here
        // I know this was called because a threshold was crossed
        if (!free_fall_detected)
        {
            free_fall_detected = true;
        }
    }
}