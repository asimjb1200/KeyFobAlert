#include <Arduino.h>
#include <audio.h>
#include <util/delay.h>
#include <mcu_state.h>

#define TIMER_PERIPHERAL TCA0
//static constexpr int CLK_FREQ = 20'000'000;

// used for the switch button that the user will press to stop the audio
void setupStopAudioPin() {
    // Set pin PA5 to input
    PORTA.DIRCLR = PIN5_bm;

    PORTA.PIN5CTRL = PORT_PULLUPEN_bm;
}

void setupShutdownBuzzerPin() {
    // set pin PB2 to output
    PORTB.DIRSET = PIN2_bm;

    // pull it low for now
    PORTB.OUT &= ~PIN2_bm;
}

void enableBuzzer() {
    PORTB.OUT |= PIN2_bm;
}

void shutdownBuzzer() {
    // pull it low for now
    PORTB.OUT &= ~PIN2_bm;
}
