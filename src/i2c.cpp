#include <iostream>

#include "dln/dln_generic.h"
#include "dln/dln_i2c_master.h"

#include "i2c.h"

void init_i2c(HDLN handle) {
    uint16_t conflict;
    DLN_RESULT result = DlnI2cMasterEnable(handle, I2C_PORT, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnI2cMasterEnable() error " << result << std::endl;
        return;
    }
}
