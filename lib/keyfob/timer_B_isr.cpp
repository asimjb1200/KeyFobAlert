#include <Arduino.h>
#include <audio.h>
#include <mcu_state.h>

ISR(TCB0_INT_vect){
    TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
    
    if (mcu_state == MCU_State_t::AUDIO_PLAYING) 
    {
        uint8_t* currentBuffer = usingBufferOne ? audioBufferOne : audioBufferTwo;
        
        DAC0.DATA = currentBuffer[bytesSent++];

        if (bytesSent >= 256) {
            bytesSent = 0;
            if (usingBufferOne) {
                usingBufferOne = false;
                bufferOneNeedsFill = true; // Signal the loop to refill Buffer 1
            } else {
                usingBufferOne = true;
                bufferTwoNeedsFill = true; // Signal the loop to refill Buffer 2
            }
        }
    }
}