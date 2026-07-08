#include <Arduino.h>
#include <audio.h>
#include <util/delay.h>
#include <mcu_state.h>

#define TIMER_PERIPHERAL TCA0
//static constexpr int CLK_FREQ = 20'000'000;

// set up the enable pin that will control whether or not the voltage booster chip is on or not
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
