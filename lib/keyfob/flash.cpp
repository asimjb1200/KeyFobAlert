#include <Arduino.h>
#include <SPI.h>
#include <flash.h>

#define FLASH_READ_STATUS_CMD 0x05

#define FLASH_SIZE_BYTES 0x200000

bool setupFlash()
{
    pinMode(FLASH_CS_PIN, OUTPUT);
    digitalWrite(FLASH_CS_PIN, HIGH); // CS idle high
    // Initialize the SPI bus
    SPI.begin();
    return verifyFlashChip();
}

bool verifyFlashChip() {
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI.transfer(0x9F);
    uint8_t manufacturerID = SPI.transfer(0x00); // Should be 0xC2 for Macronix
    uint8_t memoryType = SPI.transfer(0x00);     // Should be 0x20
    uint8_t memorySize = SPI.transfer(0x00);     // Should be 0x15
    digitalWrite(FLASH_CS_PIN, HIGH);

    if (manufacturerID != 0xC2) {
        Serial.println("ERROR: Flash chip not found or level shifter disconnected!");
        return false;
    } else {
        Serial.println("Flash chip verified");
        return true;
    }
}

bool isFlashBusy()
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(FLASH_CS_PIN, LOW);
    SPI.transfer(FLASH_READ_STATUS_CMD);
    uint8_t status = SPI.transfer(0x00); // Read status register
    digitalWrite(FLASH_CS_PIN, HIGH);    // stop communication
    SPI.endTransaction();
    return (status & 0x01); // Busy bit is bit 0
}

void waitForFlashReady()
{
    while (isFlashBusy())
    {
        delay(1); 
    }
}
