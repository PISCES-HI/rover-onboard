#ifndef BMP085DRIVER_H
#define BMP085DRIVER_H

#include "dln/dln_generic.h"

#define BMP085_ADDRESS 0x77 // this device only has one address

class Bmp085Driver {
    public:
        Bmp085Driver(HDLN handle, uint8_t addr = BMP085_ADDRESS);

        void initialize();
        bool testConnection();

        // CONTROL register methods
        uint8_t     getControl();
        void        setControl(uint8_t value);

        // MEASURE register methods
        uint16_t    getMeasurement2(); // 16-bit data
        uint32_t    getMeasurement3(); // 24-bit data
        uint8_t     getMeasureDelayMilliseconds(uint8_t mode=0);
        uint16_t    getMeasureDelayMicroseconds(uint8_t mode=0);

        // convenience methods
        void        loadCalibration();
        uint16_t    getRawTemperature();
        float       getTemperatureC();
        float       getTemperatureF();
        uint32_t    getRawPressure();
        float       getPressure();
        float       getAltitude(float pressure, float seaLevelPressure=101325);

    private:
        HDLN _handle;
        uint8_t _i2c_addr;
        uint8_t _buffer[3];

        bool calibrationLoaded;
        int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
        uint16_t ac4, ac5, ac6;
        int32_t b5;
        uint8_t measureMode;

        uint8_t read_byte(uint8_t i2c_addr, uint8_t reg_addr);
        void read_bytes(uint8_t i2c_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);
        uint8_t read_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num);
        uint8_t read_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t num_bits);
        void write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
        void write_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);
        void write_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
};

#endif // BMP085DRIVER_H
