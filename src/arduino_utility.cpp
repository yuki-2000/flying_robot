#include "arduino_utility.hpp"

void reset() {
    // Teensy 3.5(ARM)
    // http://qiita.com/edo_m18/items/a7c747c5bed600dca977
    // asm volatile("B _start");

    // ESP32
    ESP.restart();
}