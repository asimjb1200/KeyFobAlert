#include <flash.h>
#include <SPI.h>
#include <EEPROM.h>

#define READ_CMD 0x03
#define FLASH_RDSR 0x05 // Read Status Register
#define PWR_DWN_CMD 0xB9
#define WAKE_UP_CMD 0xAB

uint32_t lastMemoryAddress = 0;
static uint32_t eeLastMemoryAddress = 0;
static const uint32_t fileSize = 134387;

void waitForFlashReady() {
    uint8_t retry=0;
    uint8_t status = 0;

    do {
        digitalWrite(CS_PIN, LOW);
        SPI.transfer(FLASH_RDSR);   
       // Read Status Register command
        status = SPI.transfer(0x00); // Shift out the register value
        digitalWrite(CS_PIN, HIGH);
        delayMicroseconds(1);
        if(retry++>FLASH_BUSY_TIMEOUT)
            break;
    } while (status & 0x01);         // Mask for Bit 0 (WIP)
}

void checkFlashConnection() {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x9F);

    // the data coming back is 3 bytes: JEDEC ID (1 byte) + 2-byte device ID
    uint8_t jedecId = SPI.transfer(0x00);
    uint8_t devHigh = SPI.transfer(0x00);
    uint8_t devLow = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);

    // Combine the last two bytes into a single uint16_t (big-endian)
    uint16_t deviceId = ((uint16_t)devHigh << 8) | (uint16_t)devLow;

    Serial.print("jedecId ID: 0x"); Serial.println(jedecId, HEX);
    Serial.print("Flash ID: 0x"); Serial.println(deviceId, HEX);
    Serial.flush();
}

void getFlashElectronicInfo() {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x90);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00); //manufacturer's ID first
    uint8_t manufacturerId = SPI.transfer(0x00);
    uint8_t deviceId = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);

    Serial.print("manufacturerId: 0x"); Serial.println(manufacturerId, HEX);
    Serial.print("deviceId: 0x"); Serial.println(deviceId, HEX);
    Serial.flush();
}

void readFlash(uint32_t address, uint16_t size, uint8_t* buffer) {
    waitForFlashReady();

    digitalWrite(CS_PIN, LOW);
    
    SPI.transfer(READ_CMD); 
    
    // Send 24-bit Address
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);

    for (uint16_t i = 0; i < size; i++) {
        uint8_t dataByte = SPI.transfer(0x00); // Send in order to get real data
        
        buffer[i] = dataByte;
    }
    
    digitalWrite(CS_PIN, HIGH);
}

void readLastMemoryAddress() {
    EEPROM.get(eeLastMemoryAddress, lastMemoryAddress);
}

void saveLastMemoryAddress() {
    EEPROM.put(eeLastMemoryAddress, lastMemoryAddress);
}

void readNextDataChunk(uint16_t size, uint8_t* buffer) {
    if (lastMemoryAddress >= fileSize) {
        // End of file reached
        lastMemoryAddress = 0;
        saveLastMemoryAddress();
        return;
    }
    
    uint16_t bytesToRead = (lastMemoryAddress + size > fileSize) ? (fileSize - lastMemoryAddress) : size;
    
    readFlash(lastMemoryAddress, bytesToRead, buffer);
    
    lastMemoryAddress += bytesToRead;
    saveLastMemoryAddress();
    
    return;
}

void deepSleepFlash() {
    waitForFlashReady();

    digitalWrite(CS_PIN, LOW);
    SPI.transfer(PWR_DWN_CMD);
    digitalWrite(CS_PIN, HIGH);
}

void wakeUpFlash() {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(WAKE_UP_CMD);
    digitalWrite(CS_PIN, HIGH);
}

bool is_flash_blank(uint32_t total_size_bytes) {
    uint32_t address = 0;
    uint8_t buffer[256];
    
    // 1. Assert Chip Select (Low)
    digitalWrite(CS_PIN, LOW);
    
    // 2. Send Read Command (0x03) and 24-bit starting address (0x000000)
    SPI.transfer(READ_CMD);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    
    // 3. Continuously stream data and look for any non-0xFF byte
    for (uint32_t i = 0; i < total_size_bytes; i++) {
        uint8_t b = SPI.transfer(0x00); // Clock in next byte
        
        if (b != 0xFF) {
            // Found data! De-assert CS and exit early
            digitalWrite(CS_PIN, HIGH);
            return false; 
        }
    }
    
    // 4. De-assert Chip Select (High)
    digitalWrite(CS_PIN, HIGH);
    return true; // Entire chip is 0xFF
}
