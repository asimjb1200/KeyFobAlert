#pragma once

#include <stdbool.h>

#define FLASH_READ_CMD 0x03
#define FLASH_CS_PIN PIN_PA4

/**
 * @brief set up the MX25L1606E to be read from
 */
bool setupFlash();
bool isFlashBusy();
void waitForFlashReady();