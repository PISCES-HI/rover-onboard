#ifndef PWMDRIVER_H
#define PWMDRIVER_H

#include "dln/dln_generic.h"

class PwmDriver {
    public:
        PwmDriver(uint8_t addr = 0x40);

        void begin(HDLN handle);
        void reset(HDLN handle);
        void set_pwm_freq(HDLN handle, float freq);
        void set_pwm(HDLN handle, uint8_t num, uint16_t on, uint16_t off);
        void set_pin(HDLN handle, uint8_t num, uint16_t val, bool invert=false);

    private:
        uint8_t _i2c_addr;

        uint8_t read8(HDLN handle, uint8_t addr);
        void write8(HDLN handle, uint8_t addr, uint8_t d);
};

#endif // PWMDRIVER_H
