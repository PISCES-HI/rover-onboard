#include <iostream>
#include <cmath>
#include <unistd.h>

#include "dln/dln_generic.h"
#include "dln/dln_i2c_master.h"

#include "i2c.h"
#include "Hmc5883lDriver.h"

const uint8_t HMC5883L_RA_CONFIG_A =        0x00;
const uint8_t HMC5883L_RA_CONFIG_B =        0x01;
const uint8_t HMC5883L_RA_MODE =            0x02;
const uint8_t HMC5883L_RA_DATAX_H =         0x03;
const uint8_t HMC5883L_RA_DATAX_L =         0x04;
const uint8_t HMC5883L_RA_DATAZ_H =         0x05;
const uint8_t HMC5883L_RA_DATAZ_L =         0x06;
const uint8_t HMC5883L_RA_DATAY_H =         0x07;
const uint8_t HMC5883L_RA_DATAY_L =         0x08;
const uint8_t HMC5883L_RA_STATUS =          0x09;
const uint8_t HMC5883L_RA_ID_A =            0x0A;
const uint8_t HMC5883L_RA_ID_B =            0x0B;
const uint8_t HMC5883L_RA_ID_C =            0x0C;

const uint8_t HMC5883L_CRA_AVERAGE_BIT =    6;
const uint8_t HMC5883L_CRA_AVERAGE_LENGTH = 2;
const uint8_t HMC5883L_CRA_RATE_BIT =       4;
const uint8_t HMC5883L_CRA_RATE_LENGTH =    3;
const uint8_t HMC5883L_CRA_BIAS_BIT =       1;
const uint8_t HMC5883L_CRA_BIAS_LENGTH =    2;

const uint8_t HMC5883L_AVERAGING_1 =        0x00;
const uint8_t HMC5883L_AVERAGING_2 =        0x01;
const uint8_t HMC5883L_AVERAGING_4 =        0x02;
const uint8_t HMC5883L_AVERAGING_8 =        0x03;

const uint8_t HMC5883L_RATE_0P75 =          0x00;
const uint8_t HMC5883L_RATE_1P5 =           0x01;
const uint8_t HMC5883L_RATE_3 =             0x02;
const uint8_t HMC5883L_RATE_7P5 =           0x03;
const uint8_t HMC5883L_RATE_15 =            0x04;
const uint8_t HMC5883L_RATE_30 =            0x05;
const uint8_t HMC5883L_RATE_75 =            0x06;

const uint8_t HMC5883L_BIAS_NORMAL =        0x00;
const uint8_t HMC5883L_BIAS_POSITIVE =      0x01;
const uint8_t HMC5883L_BIAS_NEGATIVE =      0x02;

const uint8_t HMC5883L_CRB_GAIN_BIT =       7;
const uint8_t HMC5883L_CRB_GAIN_LENGTH =    3;

const uint8_t HMC5883L_GAIN_1370 =          0x00;
const uint8_t HMC5883L_GAIN_1090 =          0x01;
const uint8_t HMC5883L_GAIN_820 =           0x02;
const uint8_t HMC5883L_GAIN_660 =           0x03;
const uint8_t HMC5883L_GAIN_440 =           0x04;
const uint8_t HMC5883L_GAIN_390 =           0x05;
const uint8_t HMC5883L_GAIN_330 =           0x06;
const uint8_t HMC5883L_GAIN_220 =           0x07;

const uint8_t HMC5883L_MODEREG_BIT =        1;
const uint8_t HMC5883L_MODEREG_LENGTH =     2;

const uint8_t HMC5883L_MODE_CONTINUOUS =    0x00;
const uint8_t HMC5883L_MODE_SINGLE =        0x01;
const uint8_t HMC5883L_MODE_IDLE =          0x02;

const uint8_t HMC5883L_STATUS_LOCK_BIT =    1;
const uint8_t HMC5883L_STATUS_READY_BIT =   0;

Hmc5883lDriver::Hmc5883lDriver(HDLN handle, uint8_t addr) : _handle(handle), _i2c_addr(addr) { }

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements). Default settings include 8-sample
 * averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for mor information).
 */
void Hmc5883lDriver::initialize() {
    // write CONFIG_A register
    write_byte(_i2c_addr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    // write CONFIG_B register
    setGain(HMC5883L_GAIN_1090);
    
    // write MODE register
    setMode(HMC5883L_MODE_SINGLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool Hmc5883lDriver::testConnection() {
    read_bytes(_i2c_addr, HMC5883L_RA_ID_A, 3, _buffer);
    return (_buffer[0] == 'H' && _buffer[1] == '4' && _buffer[2] == '3');
}

// CONFIG_A register

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t Hmc5883lDriver::getSampleAveraging() {
    return read_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH);
}
/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void Hmc5883lDriver::setSampleAveraging(uint8_t averaging) {
    write_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 *
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t Hmc5883lDriver::getDataRate() {
    return read_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH);
}
/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see getDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void Hmc5883lDriver::setDataRate(uint8_t rate) {
    write_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t Hmc5883lDriver::getMeasurementBias() {
    return read_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH);
}
/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void Hmc5883lDriver::setMeasurementBias(uint8_t bias) {
    write_bits(_i2c_addr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 *
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t Hmc5883lDriver::getGain() {
    return read_bits(_i2c_addr, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH);
}
/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see getGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void Hmc5883lDriver::setGain(uint8_t gain) {
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    write_byte(_i2c_addr, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t Hmc5883lDriver::getMode() {
    return read_bits(_i2c_addr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH);
}
/** Set measurement mode.
 * @param newMode New measurement mode
 * @see getMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void Hmc5883lDriver::setMode(uint8_t newMode) {
    // use this method to guarantee that bits 7-2 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    write_byte(_i2c_addr, HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void Hmc5883lDriver::getHeading(int16_t *x, int16_t *y, int16_t *z) {
    read_bytes(_i2c_addr, HMC5883L_RA_DATAX_H, 6, _buffer);
    if (mode == HMC5883L_MODE_SINGLE)
        write_byte(_i2c_addr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    *x = (((int16_t)_buffer[0]) << 8) | _buffer[1];
    *y = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    *z = (((int16_t)_buffer[2]) << 8) | _buffer[3];
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t Hmc5883lDriver::getHeadingX() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    read_bytes(_i2c_addr, HMC5883L_RA_DATAX_H, 6, _buffer);
    if (mode == HMC5883L_MODE_SINGLE)
        write_byte(_i2c_addr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)_buffer[0]) << 8) | _buffer[1];
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t Hmc5883lDriver::getHeadingY() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    read_bytes(_i2c_addr, HMC5883L_RA_DATAX_H, 6, _buffer);
    if (mode == HMC5883L_MODE_SINGLE)
        write_byte(_i2c_addr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)_buffer[4]) << 8) | _buffer[5];
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t Hmc5883lDriver::getHeadingZ() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    read_bytes(_i2c_addr, HMC5883L_RA_DATAX_H, 6, _buffer);
    if (mode == HMC5883L_MODE_SINGLE)
        write_byte(_i2c_addr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)_buffer[2]) << 8) | _buffer[3];
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode
 * changed, two, the mode is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
bool Hmc5883lDriver::getLockStatus() {
    return read_bit(_i2c_addr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT);
}
/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
bool Hmc5883lDriver::getReadyStatus() {
    return read_bit(_i2c_addr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT);
}

// ID_* registers

/** Get identification byte A
 * @return ID_A byte (should be 01001000, ASCII value 'H')
 */
uint8_t Hmc5883lDriver::getIDA() {
    return read_byte(_i2c_addr, HMC5883L_RA_ID_A);
}
/** Get identification byte B
 * @return ID_A byte (should be 00110100, ASCII value '4')
 */
uint8_t Hmc5883lDriver::getIDB() {
    return read_byte(_i2c_addr, HMC5883L_RA_ID_B);
}
/** Get identification byte C
 * @return ID_A byte (should be 00110011, ASCII value '3')
 */
uint8_t Hmc5883lDriver::getIDC() {
    return read_byte(_i2c_addr, HMC5883L_RA_ID_C);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t Hmc5883lDriver::read_byte(uint8_t i2c_addr, uint8_t reg_addr) {
    uint8_t data;
    DLN_RESULT result = DlnI2cMasterRead(_handle, I2C_PORT, i2c_addr, 1, reg_addr, 1, &data);
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterRead() error 0x" << std::hex << result << std::endl;
        return 0;
    }
    return data;
}

void Hmc5883lDriver::read_bytes(uint8_t i2c_addr, uint8_t reg_addr, uint8_t length, uint8_t* data) {
    DLN_RESULT result = DlnI2cMasterRead(_handle, I2C_PORT, i2c_addr, 1, reg_addr, length, data);
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterRead() error 0x" << std::hex << result << std::endl;
    }
}

// Read a single bit from an 8-bit device register.
uint8_t Hmc5883lDriver::read_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num) {
    uint8_t b = read_byte(i2c_addr, reg_addr);
    return b & (1 << bit_num);
}

/** Read multiple bits from an 8-bit device register.
 * @param _i2c_addr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param num_bits Number of bits to read (not more than 8)
 * @return right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 */
uint8_t Hmc5883lDriver::read_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t num_bits) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted

    uint8_t b = read_byte(i2c_addr, reg_addr);
    uint8_t mask = ((1 << num_bits) - 1) << (bit_start - num_bits + 1);
    return (b & mask) >> (bit_start - num_bits + 1);
}

void Hmc5883lDriver::write_byte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
    DLN_RESULT result = DlnI2cMasterWrite(_handle,
                                          I2C_PORT,
                                          i2c_addr,
                                          1, reg_addr,   // i2c device register address
                                          1, &data); // data buffer
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterWrite() error 0x" << std::hex << result << std::endl;
        return;
    }
}

/** write a single bit in an 8-bit device register.
 * @param _i2c_addr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
void Hmc5883lDriver::write_bit(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data) {
    uint8_t b = read_byte(i2c_addr, reg_addr);

    b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));

    write_byte(i2c_addr, reg_addr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param _i2c_addr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
void Hmc5883lDriver::write_bits(uint8_t i2c_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value

    uint8_t b = read_byte(i2c_addr, reg_addr);

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    write_byte(i2c_addr, reg_addr, b);
}

