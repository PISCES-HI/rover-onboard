#ifndef ANALOG_H
#define ANALOG_H

#include "dln/dln_generic.h"

void init_analog(HDLN handle);
void cleanup_analog(HDLN handle);

float read_voltage(const uint16_t analog_reading, const float r1, const float r2);
float get_48v_voltage(const uint16_t analog_reading);
float get_12v_e_voltage(const uint16_t analog_reading);
float get_12v_pl_voltage(const uint16_t analog_reading);

float get_avionics_temperature(const uint16_t analog_reading);
float get_ambient_temperature(const uint16_t analog_reading);

#endif // ANALOG_H
