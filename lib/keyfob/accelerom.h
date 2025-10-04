#pragma once
// LIS2DH12
#include <stdint.h>

extern volatile bool free_fall_detected;
//extern volatile bool impact_detected;
enum AccelConfigError {
  ACCEL_SUCCESS = 0,
  ACCEL_CTRL_REG1_FAILED = -1,
  ACCEL_CTRL_REG4_FAILED = -2,
  ACCEL_CTRL_REG3_FAILED = -3,
  ACCEL_INT1_THS_FAILED = -4,
  ACCEL_INT1_DURATION_FAILED = -5,
  ACCEL_INT1_CFG_FAILED = -6,
  ACCEL_CTRL_REG5_FAILED = -7
};

/**
 * @brief set up the interrupt PIN for the LIS2DH12
 */
void accelInterruptPinSetup();
/**
 * @brief check for the free fall and impact events
 */
void accelCheckForInterruptEvents();
/**
 * @brief sets all of the necessary registers to get meaningful data from the LIS2DH12 and to choose what events will generate interrupts
 */
int accelRegisterConfig();
void deviceRecovered();
