#ifndef ACTUATOR_HEADER_FILE
#define ACTUATOR_HEADER_FILE
#include "design3.hpp"
#include "utility.hpp"

void actuator_init();
void move_actuator(const OutputInfo&);
void servo_at_zero();

#endif