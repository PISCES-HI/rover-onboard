#ifndef HMC5883LDRIVER_H
#define HMC5883LDRIVER_H

#include "dln/dln_generic.h"

#define HMC5883L_ADDRESS            0x1E // this device only has one address

class Hmc5883lDriver {
    public:
        Hmc5883lDriver(HDLN handle, uint8_t addr = HMC5883L_ADDRESS);

        void initialize();
        bool testConnection();

        // CONFIG_A register
        uint8_t getSampleAveraging();
        void setSampleAveraging(uint8_t averaging);
        uint8_t getDataRate();
        void setDataRate(uint8_t rate);
        uint8_t getMeasurementBias();
        void setMeasurementBias(uint8_t bias);

        // CONFIG_B register
        uint8_t getGain();
        void setGain(uint8_t gain);

        // MODE register
        uint8_t getMode();
        void setMode(uint8_t mode);

        // DATA* registers
        void getHeading(int16_t *x, int16_t *y, int16_t *z);
        int16_t getHeadingX();
        int16_t getHeadingY();
        int16_t getHeadingZ();

        // STATUS register
        bool getLockStatus();
        bool getReadyStatus();

        // ID_* registers
        uint8_t getIDA();
        uint8_t getIDB();
        uint8_t getIDC();

    private:
        HDLN _handle;
        uint8_t _i2c_addr;
        uint8_t mode;
        uint8_t _buffer[6];

        uint8_t read_byte(uint8_t i2c_addr, uint8_t reg_addr);
        void read_bytes(uint8_t i2c_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);
        uint8_t read_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num);
        uint8_t read_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t num_bits);
        void write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
        void write_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);
        void write_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
};

#endif // HMC5883LDRIVER_H
