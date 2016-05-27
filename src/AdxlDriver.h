#ifndef ADXLDRIVER_H
#define ADXLDRIVER_H

#include "dln/dln_generic.h"

#define ADXL345_ADDRESS_ALT_LOW 0x53 // alt address pin low (GND)
#define ADXL345_ADDRESS_ALT_HIGH 0x1D // alt address pin high (VCC)
#define ADXL345_DEFAULT_ADDRESS ADXL345_ADDRESS_ALT_LOW

class AdxlDriver {
    public:
        AdxlDriver(HDLN handle, uint8_t addr = ADXL345_DEFAULT_ADDRESS);

        void initialize();
        bool testConnection();

        // DEVID register
        uint8_t getDeviceID();
        
        // THRESH_TAP register
        uint8_t getTapThreshold();
        void setTapThreshold(uint8_t threshold);

        // OFS* registers
        void getOffset(int8_t* x, int8_t* y, int8_t* z);
        void setOffset(int8_t x, int8_t y, int8_t z);
        int8_t getOffsetX();
        void setOffsetX(int8_t x);
        int8_t getOffsetY();
        void setOffsetY(int8_t y);
        int8_t getOffsetZ();
        void setOffsetZ(int8_t z);
        
        // DUR register
        uint8_t getTapDuration();
        void setTapDuration(uint8_t duration);
        
        // LATENT register
        uint8_t getDoubleTapLatency();
        void setDoubleTapLatency(uint8_t latency);
        
        // WINDOW register
        uint8_t getDoubleTapWindow();
        void setDoubleTapWindow(uint8_t window);
        
        // THRESH_ACT register
        uint8_t getActivityThreshold();
        void setActivityThreshold(uint8_t threshold);
        
        // THRESH_INACT register
        uint8_t getInactivityThreshold();
        void setInactivityThreshold(uint8_t threshold);

        // TIME_INACT register
        uint8_t getInactivityTime();
        void setInactivityTime(uint8_t time);
        
        // ACT_INACT_CTL register
        bool getActivityAC();
        void setActivityAC(bool enabled);
        bool getActivityXEnabled();
        void setActivityXEnabled(bool enabled);
        bool getActivityYEnabled();
        void setActivityYEnabled(bool enabled);
        bool getActivityZEnabled();
        void setActivityZEnabled(bool enabled);
        bool getInactivityAC();
        void setInactivityAC(bool enabled);
        bool getInactivityXEnabled();
        void setInactivityXEnabled(bool enabled);
        bool getInactivityYEnabled();
        void setInactivityYEnabled(bool enabled);
        bool getInactivityZEnabled();
        void setInactivityZEnabled(bool enabled);
        
        // THRESH_FF register
        uint8_t getFreefallThreshold();
        void setFreefallThreshold(uint8_t threshold);
        
        // TIME_FF register
        uint8_t getFreefallTime();
        void setFreefallTime(uint8_t time);
        
        // TAP_AXES register
        bool getTapAxisSuppress();
        void setTapAxisSuppress(bool enabled);
        bool getTapAxisXEnabled();
        void setTapAxisXEnabled(bool enabled);
        bool getTapAxisYEnabled();
        void setTapAxisYEnabled(bool enabled);
        bool getTapAxisZEnabled();
        void setTapAxisZEnabled(bool enabled);
        
        // ACT_TAP_STATUS register
        bool getActivitySourceX();
        bool getActivitySourceY();
        bool getActivitySourceZ();
        bool getAsleep();
        bool getTapSourceX();
        bool getTapSourceY();
        bool getTapSourceZ();
        
        // BW_RATE register
        bool getLowPowerEnabled();
        void setLowPowerEnabled(bool enabled);
        uint8_t getRate();
        void setRate(uint8_t rate);

        // POWER_CTL register
        bool getLinkEnabled();
        void setLinkEnabled(bool enabled);
        bool getAutoSleepEnabled();
        void setAutoSleepEnabled(bool enabled);
        bool getMeasureEnabled();
        void setMeasureEnabled(bool enabled);
        bool getSleepEnabled();
        void setSleepEnabled(bool enabled);
        uint8_t getWakeupFrequency();
        void setWakeupFrequency(uint8_t frequency);
        
        // INT_ENABLE register
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);
        bool getIntSingleTapEnabled();
        void setIntSingleTapEnabled(bool enabled);
        bool getIntDoubleTapEnabled();
        void setIntDoubleTapEnabled(bool enabled);
        bool getIntActivityEnabled();
        void setIntActivityEnabled(bool enabled);
        bool getIntInactivityEnabled();
        void setIntInactivityEnabled(bool enabled);
        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntWatermarkEnabled();
        void setIntWatermarkEnabled(bool enabled);
        bool getIntOverrunEnabled();
        void setIntOverrunEnabled(bool enabled);
        
        // INT_MAP register
        uint8_t getIntDataReadyPin();
        void setIntDataReadyPin(uint8_t pin);
        uint8_t getIntSingleTapPin();
        void setIntSingleTapPin(uint8_t pin);
        uint8_t getIntDoubleTapPin();
        void setIntDoubleTapPin(uint8_t pin);
        uint8_t getIntActivityPin();
        void setIntActivityPin(uint8_t pin);
        uint8_t getIntInactivityPin();
        void setIntInactivityPin(uint8_t pin);
        uint8_t getIntFreefallPin();
        void setIntFreefallPin(uint8_t pin);
        uint8_t getIntWatermarkPin();
        void setIntWatermarkPin(uint8_t pin);
        uint8_t getIntOverrunPin();
        void setIntOverrunPin(uint8_t pin);

        // INT_SOURCE register
        uint8_t getIntDataReadySource();
        uint8_t getIntSingleTapSource();
        uint8_t getIntDoubleTapSource();
        uint8_t getIntActivitySource();
        uint8_t getIntInactivitySource();
        uint8_t getIntFreefallSource();
        uint8_t getIntWatermarkSource();
        uint8_t getIntOverrunSource();
        
        // DATA_FORMAT register
        uint8_t getSelfTestEnabled();
        void setSelfTestEnabled(uint8_t enabled);
        uint8_t getSPIMode();
        void setSPIMode(uint8_t mode);
        uint8_t getInterruptMode();
        void setInterruptMode(uint8_t mode);
        uint8_t getFullResolution();
        void setFullResolution(uint8_t resolution);
        uint8_t getDataJustification();
        void setDataJustification(uint8_t justification);
        uint8_t getRange();
        void setRange(uint8_t range);

        // DATA* registers
        void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAccelerationX();
        int16_t getAccelerationY();
        int16_t getAccelerationZ();

        // FIFO_CTL register
        uint8_t getFIFOMode();
        void setFIFOMode(uint8_t mode);
        uint8_t getFIFOTriggerInterruptPin();
        void setFIFOTriggerInterruptPin(uint8_t interrupt);
        uint8_t getFIFOSamples();
        void setFIFOSamples(uint8_t size);
        
        // FIFO_STATUS register
        bool getFIFOTriggerOccurred();
        uint8_t getFIFOLength();

    private:
        HDLN _handle;
        uint8_t _i2c_addr;
        uint8_t _buffer[6];

        uint8_t read_byte(uint8_t i2c_addr, uint8_t reg_addr);
        void read_bytes(uint8_t i2c_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);
        uint8_t read_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num);
        uint8_t read_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t num_bits);
        void write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
        void write_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);
        void write_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
};

#endif // ADXLDRIVER_H
