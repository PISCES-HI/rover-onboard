#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>

double get_thermistor_temp(uint16_t adc_reading);

#endif // THERMISTOR_H
