#ifndef I2C_H
#define I2C_H

#include "dln/dln_generic.h"

const unsigned int I2C_PORT = 0;
const unsigned int PWM_I2C_ADDR = 0x40;

void init_i2c(HDLN handle);

#endif // I2C_H
