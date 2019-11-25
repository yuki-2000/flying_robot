#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>
#include "design3.hpp"

const size_t SERIAL_PARSER_BUFFER = 64;

class SerialParser {
   public:
    SerialParser() { buff_len_ = 0; }

    // read a line from serial. if reading was failed, return false.
    bool read_line();

    Command parse_command();

    float parse_float();

    void flush() { buff_len_ = 0; }

   private:
    size_t buff_len_;
    char buff_[SERIAL_PARSER_BUFFER];
};

#endif
