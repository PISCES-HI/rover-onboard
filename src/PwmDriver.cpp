#include <iostream>
#include <cmath>
#include <unistd.h>

#include "dln/dln_generic.h"
#include "dln/dln_i2c_master.h"

#include "i2c.h"
#include "PwmDriver.h"

// Set to true to print some debug messages, or false to disable them.
const bool ENABLE_DEBUG_OUTPUT = true;

const uint8_t PCA9685_SUBADR1 = 0x2;
const uint8_t PCA9685_SUBADR2 = 0x3;
const uint8_t PCA9685_SUBADR3 = 0x4;

const uint8_t PCA9685_MODE1 = 0x0;
const uint8_t PCA9685_PRESCALE = 0xFE;

const uint8_t LED0_ON_L = 0x6;
const uint8_t LED0_ON_H = 0x7;
const uint8_t LED0_OFF_L = 0x8;
const uint8_t LED0_OFF_H = 0x9;

const uint8_t ALLLED_ON_L = 0xFA;
const uint8_t ALLLED_ON_H = 0xFB;
const uint8_t ALLLED_OFF_L = 0xFC;
const uint8_t ALLLED_OFF_H = 0xFD;

PwmDriver::PwmDriver(uint8_t addr) : _i2c_addr(addr) { }

void PwmDriver::begin(HDLN handle) {
    this->reset(handle);
}


void PwmDriver::reset(HDLN handle) {
    write8(handle, PCA9685_MODE1, 0x0);
}

void PwmDriver::set_pwm_freq(HDLN handle, float freq) {
    //Serial.print("Attempting to set freq ");
    //Serial.println(freq);
    freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    if (ENABLE_DEBUG_OUTPUT) {
        std::cout << "Estimated pre-scale: " << prescaleval << std::endl;
    }
    uint8_t prescale = floor(prescaleval + 0.5);
    if (ENABLE_DEBUG_OUTPUT) {
        std::cout << "Final pre-scale: " << prescale << std::endl;
    }

    uint8_t oldmode = read8(handle, PCA9685_MODE1);
    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
    this->write8(handle, PCA9685_MODE1, newmode); // go to sleep
    this->write8(handle, PCA9685_PRESCALE, prescale); // set the prescaler
    this->write8(handle, PCA9685_MODE1, oldmode);
    usleep(5*1000); // Sleep for 5 milliseconds
    this->write8(handle, PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                            // This is why the beginTransmission below was not working.
    //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void PwmDriver::set_pwm(HDLN handle, uint8_t num, uint16_t on, uint16_t off) {
    //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

    /*WIRE.beginTransmission(_i2c_addr);
    WIRE.write(LED0_ON_L+4*num);
    WIRE.write(on);
    WIRE.write(on>>8);
    WIRE.write(off);
    WIRE.write(off>>8);
    WIRE.endTransmission();*/

    uint8_t data[] = { on, on>>8, off, off>>8 };

    DLN_RESULT result = DlnI2cMasterWrite(handle,
                                          I2C_PORT,
                                          _i2c_addr,
                                          1, LED0_ON_L+4*num, // i2c device internal address
                                          4, (uint8_t*)&data); // data buffer
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterWrite() returned 0x" << result << std::endl;
        return;
    }
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void PwmDriver::set_pin(HDLN handle, uint8_t num, uint16_t val, bool invert) {
    // Clamp value between 0 and 4095 inclusive.
    val = std::min((uint16_t)val, (uint16_t)4095);
    if (invert) {
        if (val == 0) {
            // Special value for signal fully on.
            this->set_pwm(handle, num, 4096, 0);
        }
        else if (val == 4095) {
            // Special value for signal fully off.
            this->set_pwm(handle, num, 0, 4096);
        }
        else {
            this->set_pwm(handle, num, 0, 4095-val);
        }
    }
    else {
        if (val == 4095) {
            // Special value for signal fully on.
            this->set_pwm(handle, num, 4096, 0);
        }
        else if (val == 0) {
            // Special value for signal fully off.
            this->set_pwm(handle, num, 0, 4096);
        }
        else {
            this->set_pwm(handle, num, 0, val);
        }
    }
}

uint8_t PwmDriver::read8(HDLN handle, uint8_t addr) {
    uint8_t data;
    DLN_RESULT result = DlnI2cMasterRead(handle, I2C_PORT, _i2c_addr, 1, addr, 1, &data);
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterRead() returned 0x" << result << std::endl;
        return 0;
    }
    return data;

    /*WIRE.beginTransmission(_i2c_addr);
    WIRE.write(addr);
    WIRE.endTransmission();

    WIRE.requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
    return WIRE.read();*/
}

void PwmDriver::write8(HDLN handle, uint8_t addr, uint8_t data) {
    DLN_RESULT result = DlnI2cMasterWrite(handle,
                                          I2C_PORT,
                                          _i2c_addr,
                                          1, addr, // i2c device internal address
                                          1, &data); // data buffer
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterWrite() returned 0x" << result << std::endl;
        return;
    }

    /*WIRE.beginTransmission(_i2c_addr);
    WIRE.write(addr);
    WIRE.write(d);
    WIRE.endTransmission();*/
}
