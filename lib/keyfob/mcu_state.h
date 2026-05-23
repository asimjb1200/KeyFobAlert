#pragma once

typedef enum {
    RESTING, 
    FALL_DETECTED,
    AUDIO_PLAYING,
    DEVICE_RECOVERED,
    CHARGING // need to create an interrupt that informs me when the battery is full
} MCU_State_t;

extern MCU_State_t mcu_state;