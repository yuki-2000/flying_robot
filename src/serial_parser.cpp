#include "serial_parser.h"
#include <cstdlib>
#include <cstring>

bool SerialParser::read_line() {
    while (Serial.available()) {
        buff_[buff_len_] = Serial.read();
        buff_len_ = buff_len_ + 1 == SERIAL_PARSER_BUFFER ? 0 : buff_len_ + 1;

        if (buff_len_ > 2 && buff_[buff_len_ - 2] == '\r' &&
            buff_[buff_len_ - 1] == '\n') {
            buff_[buff_len_] = '\0';
            return true;
        }
    }

    return false;
}

Command SerialParser::parse_command() {
    buff_len_ = 0;

    if (strcmp(buff_, "RESET\r\n") == 0)
        return COMMAND_RESET;
    else if (strcmp(buff_, "BIAS\r\n") == 0)
        return COMMAND_BIAS;
    else
        return COMMAND_NONE;
}

float SerialParser::parse_float() {
    buff_len_ = 0;
    return atof(buff_);
}
