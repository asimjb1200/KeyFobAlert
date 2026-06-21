#pragma once

enum MCU_State_t :uint8_t {
    RESTING = 0, 
    FALL_DETECTED = 1,
    AUDIO_PLAYING = 2,
    DEVICE_RECOVERED = 3,
    CHARGING = 4
};

extern volatile MCU_State_t mcu_state;