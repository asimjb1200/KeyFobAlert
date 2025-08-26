#pragma once
// LIS2DH12
#include <stdint.h>

extern volatile bool free_fall_detected;
extern volatile bool impact_detected;
/**
 * @brief set up the interrupt PIN for the LIS2DH12
 */
void accelInterruptSetup();
/**
 * @brief check for the free fall and impact events
 */
void accelCheckForInterruptEvents();
/**
 * @brief sets all of the necessary registers to get meaningful data from the LIS2DH12 and to choose what events will generate interrupts
 */
void accelRegisterConfig();
