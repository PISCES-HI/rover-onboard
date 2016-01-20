#include "dln_generic.h"
#include "dln_i2c_master.h"

const unsigned int I2C_PORT = 0;
const unsigned int PWM_I2C_ADDR = 0x40;

void init_pwm_board(HDLN handle) {
    uint16_t conflict;
    DLN_RESULT result = DlnI2cMasterEnable(handle, I2C_PORT, &conflict);
    if (DLN_FAILED(result)) {
        QMessageBox::warning(this, tr("DlnI2cMasterEnable() failed"),
                                   tr("DlnI2cMasterEnable() function returns 0x")+ QString::number(result, 16).toUpper());
        return;
    }
}

void set_camera_pan(double angle) {
}

void set_camera_tilt(double angle) {
}
