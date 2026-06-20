#pragma once

typedef enum {
    RESTING = 0, 
    FALL_DETECTED = 1,
    AUDIO_PLAYING = 2,
    DEVICE_RECOVERED = 3,
    CHARGING = 4
} MCU_State_t;

extern volatile MCU_State_t mcu_state;