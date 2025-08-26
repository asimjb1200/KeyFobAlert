#pragma once

#include <stdint.h>
#include <stdbool.h>

// Buffer management variables
extern volatile bool buffer_A_needs_refill;
extern volatile bool buffer_B_needs_refill;
extern volatile bool using_buffer_A;  // Which buffer DAC is currently using
extern volatile bool audio_playing;

/**
 * @brief Checks if any buffers needs refilling and refills them if necessary
 */
void checkAndFillEmptyAudioBuffer();

/**
 * @brief configure the interrupt for sampling audio and pre fill the two buffers
 */
void setupAudioInterruptTimer();
void disableAudio();
/**
 * @brief Enable the overflow interrupt for TCA0, which is what allows the ISR
 * to run and feed audio data samples to the speaker, I.E. plays the audio
 */
void enableAudio();
