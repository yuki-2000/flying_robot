#ifndef ARDUINO_UTILITY_H
#define ARDUINO_UTILITY_H

#include "Arduino.h"
#include "EEPROM.h"

// function to write data to EEPROM
template <typename T>
void write_to_eeprom(const size_t address_offset, T data) {
    char *buff = (char *)(&data);

    for (size_t i = 0; i < sizeof(T); i++) {
        EEPROM.write(address_offset + i, buff[i]);
    }
}

// function to read from EEPROM
template <typename T>
T read_from_eeprom(const size_t address_offset) {
    T data;
    char *buff = (char *)(&data);

    for (size_t i = 0; i < sizeof(T); i++) {
        buff[i] = EEPROM.read(address_offset + i);
    }

    return data;
}

void reset();

#endif