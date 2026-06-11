#pragma once
#include <stdint.h>

#define CS_PIN PIN_PA7
#define FLASH_BUSY_TIMEOUT  200

void waitForFlashReady();
void readLastMemoryAddress();
void saveLastMemoryAddress();
void readFlash(uint32_t address, uint16_t size, uint8_t* buffer);
void readNextDataChunk(uint16_t size, uint8_t* buffer);
void deepSleepFlash();
void wakeUpFlash();
void checkFlashConnection();
void getFlashElectronicInfo();