#include <stdio.h>

#include "dln/dln.h"
#include "dln/dln_generic.h"
#include "dln/dln_gpio.h"
//#include "dln/dln_i2c_master.h"

#include "i2c.h"
#include "PwmDriver.h"
#include "RoverControl.h"

int main() {
    DLN_RESULT result;

    DlnConnect("localhost", DLN_DEFAULT_SERVER_PORT);

    // Check device count
    uint32_t device_count;
    result = DlnGetDeviceCount(&device_count);
    if (!DLN_SUCCEEDED(result)) {
        printf("Failed to get DLN device count");
        return -1;
    }
    if (device_count != 1) {
        printf("There should be one and only one DLN device connected. There are %d", device_count);
        return -1;
    }

    // Try to open our device
    HDLN handle;
    result = DlnOpenDevice(0, &handle);
    if (!DLN_SUCCEEDED(result)) {
        printf("Failed to open DLN device");
        return -1;
    }

    // Print out some info about our DLN device
    DLN_VERSION version;
    uint32_t sn, id;
    DlnGetVersion(handle, &version);
    DlnGetDeviceSn(handle, &sn);
    DlnGetDeviceId(handle, &id);
    printf("Opened DLN device: %d\t%d\n", sn, id);

    uint16_t pin = 0;
    DLN_RESULT enable_result = DlnGpioPinEnable(handle, pin);
    if (DLN_FAILED(enable_result))
    {
        printf("DlnGpioPinEnable failed\n");
    }
    DLN_RESULT dir_result = DlnGpioPinSetDirection(handle, pin, 1);
    if (DLN_FAILED(dir_result))
    {
        printf("DlnGpioPinSetDirection failed\n");
    }
    uint8_t value = 0;
    DLN_RESULT val0_result = DlnGpioPinGetVal(handle, 0, &value);
    printf("P0: %d\n", value);
    DLN_RESULT val1_result = DlnGpioPinGetVal(handle, 1, &value);
    printf("P1: %d\n", value);
    DLN_RESULT val2_result = DlnGpioPinGetVal(handle, 2, &value);
    printf("P2: %d\n", value);
    DlnGpioPinSetOutVal(handle, 0, 1);
    DlnGpioPinSetOutVal(handle, 1, 0);
    DlnGpioPinSetOutVal(handle, 2, 0);

    init_i2c(handle);

    PwmDriver pwm;
    pwm.begin(handle);
    pwm.set_pwm_freq(handle, 50);
    pwm.set_pin(handle, 0, 512);

    DlnCloseHandle(handle);

    DlnDisconnectAll();

    return 0;
}
