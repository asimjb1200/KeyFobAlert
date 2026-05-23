#include <mcu_state.h>
#include <Arduino.h>

ISR(PORTA_PORT_vect) {
    // Check to see if the interrupt came from the audio sht down pin
    if (PORTA.INTFLAGS & PIN5_bm) {
        PORTA.INTFLAGS &= ~PIN5_bm;

        mcu_state = DEVICE_RECOVERED;
        return;
    }

    // interrupt came from accelerometer
    if (PORTA.INTFLAGS & PIN4_bm) {
        PORTA.INTFLAGS &= ~PIN4_bm;

        // Disable the interrupt
        PORTA.PIN4CTRL &= ~PORT_ISC_gm;
        PORTA.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;

        // works better when I call it here..
        //readRegister(ACCELEROMETER_ADDR, INT1_SRC_REGISTER);

        mcu_state = FALL_DETECTED;
        return;
    }
}