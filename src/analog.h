#ifndef ANALOG_H
#define ANALOG_H

#include "dln/dln_generic.h"

const unsigned int I2C_PORT = 0;
const unsigned int PWM_I2C_ADDR = 0x40;

void init_analog(HDLN handle);
void cleanup_analog(HDLN handle);

#endif // ANALOG_H
