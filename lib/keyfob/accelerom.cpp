#include <Wire.h>

#define ACCEL_INT_PIN PIN_PB3

#define ACCEL_ADDRESS 0x18
#define ACCEL_CTRL_REG1 0X20
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
volatile bool impact_detected = false;

// TODO: Make sure the bitmask for pin PB3 is correct
void accelInterruptSetup()
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

// TODO: Check the data sheets for a compatible BAUD rate
void twiSetup()
{
    accelRegisterConfig();
}

void accelRegisterConfig()
{
    // set data rate to 100Hz and enable all axis
    uint8_t dataRateConfig = 0b01010111;

    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG1);
    Wire.write(dataRateConfig);
    Wire.endTransmission();

    // block update, little endian, +/-2g resolution, normal operating mode
    uint8_t ctrlReg4Config = 0b10000000;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG4);
    Wire.write(ctrlReg4Config);
    Wire.endTransmission();

    // I want my threshold to be 0.4g
    // need to convert it to milligrams to work with the accel register
    // 0.4g * 1000mg/g = 400mg - g force in mg
    // divide by the register factor to give it increments that it understands
    // 400mg/16mg LSB = 25
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_THS);
    Wire.write(25);
    Wire.endTransmission();

    // set the duration needed before the interrupt event is considered valid
    // I want 50ms so since 1 LSB = 1/0DR ~ 10ms I can use 5 as the multiplier
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_DURATION);
    Wire.write(5);
    Wire.endTransmission();

    // interrupts generate on x, y & z axes
    uint8_t int_cfg_reg = 0b00111111;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_CFG);
    Wire.write(int_cfg_reg);
    Wire.endTransmission();

    // activate the interrupts on physical INT1 pin
    // INTERRUPT ACTIVE = IA
    uint8_t ctrl_reg3_cfg = 0b01000000; // I1_IA1 = 1
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG3);
    Wire.write(ctrl_reg3_cfg);
    Wire.endTransmission();

    // With latching enabled, once the interrupt pin goes high,
    // it stays high until you read the source register (INT1_SRC at 0x31)
    uint8_t ctrl_reg5_cfg = 0b00001000;
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_CTRL_REG5);
    Wire.write(ctrl_reg5_cfg);
    Wire.endTransmission();
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
    if (free_fall_detected || impact_detected)
    {
        // process force data
        determineEvent();
    }
    else
    {
        return;
    }
}

// Determine the Event:
//  If the magnitude is low (e.g., less than 0.2g), likely a free-fall event.
//  If the magnitude is high (e.g., greater than 2g) impact event
static void determineEvent()
{
    int16_t raw_x, raw_y, raw_z;
    readFromAccel(raw_x, raw_y, raw_z);

    // make sense of the readings to distinguish between free fall and impact based on g force

    // Right-Shift for 10-bit Data
    int16_t shifted_x = raw_x >> 6;
    int16_t shifted_y = raw_y >> 6;
    int16_t shifted_z = raw_z >> 6;

    float x_g = (float)shifted_x * 4.0 / 1000.0;
    float y_g = (float)shifted_y * 4.0 / 1000.0;
    float z_g = (float)shifted_z * 4.0 / 1000.0;

    float magnitude = sqrt(x_g * x_g + y_g * y_g + z_g * z_g);

    // Check the magnitude to determine the type of event
    if (magnitude <= 0.5 && magnitude > 0.1) // A low-g threshold for free-fall
    {
        free_fall_detected = true;
        impact_detected = false; // Reset impact flag for new sequence
    }
    else if (magnitude >= 2.0 && free_fall_detected)
    {
        impact_detected = true;
    }
    else
    {
        free_fall_detected = false;
        impact_detected = false;
    }

    // Read the source register so that the pin can go back low
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(ACCEL_INT1_SRC);
    Wire.endTransmission(false);
    Wire.requestFrom(ACCEL_ADDRESS, 1);
}

// TODO: ADD ERROR HANDLING TO ACCELERATOR READING
static void readFromAccel(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z)
{
    Wire.beginTransmission(ACCEL_ADDRESS);

    //    To enable address auto-increment, set the MSB of the register address to 1.
    //    0x28 becomes 0x28 | 0x80 = 0xA8
    Wire.write(OUT_X_L | 0x80); // Start reading at OUT_X_L and automatically increment
    Wire.endTransmission(false);

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
    }
}

void deviceRecovered() {
    free_fall_detected = false;
    impact_detected = false;
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
        else if (free_fall_detected && !impact_detected)
        {
            impact_detected = true;
        }
    }
}