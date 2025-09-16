#include <Arduino.h>
#include <audio.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <flash.h>

#define TIMER_PERIPHERAL TCA0
#define SAMPLE_RATE_HZ 44100UL // 44.1 kHz
#define AUDIO_DATA_START 0x000000
#define AUDIO_DATA_SIZE (1024 * 1024)
#define AUDIO_DATA_END (AUDIO_DATA_START + AUDIO_DATA_SIZE - 1) // 0x0FFFFF
#define AUDIO_BUFFER_SIZE 128
#define F_CPU 20000000UL       // 20 MHz


volatile uint8_t current_sample_index = 0;
volatile bool buffer_A_needs_refill = false;
volatile bool buffer_B_needs_refill = false;
volatile bool using_buffer_A = true;
volatile bool audio_playing = false;
volatile bool request_audio_stop = false;

// start reading from the very beginning of flash memory.
static uint32_t current_flash_address = AUDIO_DATA_START; // Track position in flash

// Double buffer setup
static uint8_t audio_buffer_A[AUDIO_BUFFER_SIZE];
static uint8_t audio_buffer_B[AUDIO_BUFFER_SIZE];

static void initializeDAC()
{
    // set the voltage range
    VREF.CTRLA = VREF_DAC0REFSEL_2V5_gc;

    // enable the DAC for outputting voltage thru DAC pin
    DAC0.CTRLA = DAC_OUTEN_bm | DAC_ENABLE_bm;

    // set initial data value
    DAC0.DATA = 128;
}

void setupAudioInterruptTimer()
{
    initializeDAC();
    
    uint16_t interruptPeriod = (F_CPU / (1 * SAMPLE_RATE_HZ)) - 1;

    // Configure TCA0 for normal operation and set the period (TOP)
    // TCA0.SINGLE.CTRLA: Control A register for TCA0 (Single mode)
    // CLKSEL_DIV1_gc: No prescaling (20 MHz clock directly)
    // ENABLE_bm: Enable the TCA0 peripheral
    TIMER_PERIPHERAL.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
    TIMER_PERIPHERAL.SINGLE.PER = interruptPeriod;

    loadAudioBuffer(audio_buffer_A, AUDIO_BUFFER_SIZE);
    loadAudioBuffer(audio_buffer_B, AUDIO_BUFFER_SIZE);
}

void stopAudioSwitchInterruptSetup()
{
    // set pin PA5 as input
    PORTA.DIRCLR = (1 << 5); ; // 0b00100000

    /*
    * Enable the internal pull-up resistor and set interrupt to trigger on a falling edge
    * since pressing the switch will pull the line low
    */ 
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    PORTA.INTFLAGS = (1 << 5);// clear any stale flags
}

/*
interrupt vector for the Timer/Counter Type A (TCA)
used to trigger an action every time the timer reaches its maximum count and "overflows" to zero
*/ 
ISR(TCA0_OVF_vect)
{
    uint8_t *buffer_in_use = getCurrentAudioBuffer();

    // Get the next audio sample from the active buffer
    uint8_t next_audio_sample = buffer_in_use[current_sample_index];

    // Output it to the DAC
    DAC0.DATA = next_audio_sample;

    // Increment playback index
    current_sample_index++;

    // Check for buffer end and flip active buffer flag
    if (current_sample_index == AUDIO_BUFFER_SIZE)
    {
        // Signal main loop to fill the other buffer
        swapBuffers();
        current_sample_index = 0;
    }

    // Clear the interrupt flag:
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void checkAndFillEmptyAudioBuffer()
{
    if (buffer_B_needs_refill && using_buffer_A)
    {
        loadAudioBuffer(audio_buffer_B, AUDIO_BUFFER_SIZE);

        noInterrupts();
        buffer_B_needs_refill = false;
        interrupts();
    }
    else if (buffer_A_needs_refill && !using_buffer_A)
    {
        loadAudioBuffer(audio_buffer_A, AUDIO_BUFFER_SIZE);

        noInterrupts();
        buffer_A_needs_refill = false;
        interrupts();
    }
}

static void loadAudioBuffer(uint8_t *buffer, uint16_t size)
{
    // Check if we would go past end of audio file
    if (current_flash_address + size > AUDIO_DATA_END)
    {
        // Loop back to beginning of audio
        current_flash_address = AUDIO_DATA_START;
    }

    waitForFlashReady(); 

    digitalWrite(FLASH_CS_PIN, LOW);

    // Send READ command + address
    SPI.transfer(FLASH_READ_CMD);
    SPI.transfer((current_flash_address >> 16) & 0xFF);
    SPI.transfer((current_flash_address >> 8) & 0xFF);
    SPI.transfer(current_flash_address & 0xFF);

    // Read audio data into buffer
    for (uint16_t i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }

    digitalWrite(FLASH_CS_PIN, HIGH);

    // Update flash address for next read
    current_flash_address += size;
}

// This would be called when current buffer is nearly empty
static void swapBuffers()
{
    if (using_buffer_A)
    {
        // Switch to buffer B, reload buffer A in background
        noInterrupts();
        using_buffer_A = false;
        buffer_A_needs_refill = true;
        interrupts();
    }
    else
    {
        // Switch to buffer A, reload buffer B in background
        noInterrupts();
        using_buffer_A = true;
        buffer_B_needs_refill = true;
        interrupts();
    }
}

static uint8_t *getCurrentAudioBuffer()
{
    if (using_buffer_A && !buffer_A_needs_refill)
    {
        return audio_buffer_A;
    }
    else if (!using_buffer_A && !buffer_B_needs_refill)
    {
        return audio_buffer_B;
    }
    else
    {
        // Buffer not ready, return a safe fallback (e.g., silence)
        static uint8_t silence_buffer[AUDIO_BUFFER_SIZE] = {0};
        return silence_buffer;
    }
}

void pauseAudio(void) {
    // Disable the timer interrupt that drives audio playback
    TIMER_PERIPHERAL.SINGLE.INTCTRL &= ~TCA_SINGLE_OVF_bm; // Disable overflow interrupt
    
    // Optional: Set DAC to mid-scale to avoid pop/click
    DAC0.DATA = 128;  // 1.25V - silent level
    audio_playing = false;
}

void enableAudio(void) {
    // Enable the overflow interrupt for TCA0
    // TCA0.SINGLE.INTCTRL: Interrupt Control register for TCA0 (Single mode)
    // OVF_bm: Overflow Interrupt Enable bit
    TIMER_PERIPHERAL.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    audio_playing = true;
}