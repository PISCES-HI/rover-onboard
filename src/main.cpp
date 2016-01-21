#include <stdio.h>

#include "dln/dln.h"
#include "dln/dln_generic.h"
#include "dln/dln_gpio.h"
//#include "dln/dln_i2c_master.h"

#include "i2c.h"
#include "PwmDriver.h"

int main() {
    /*HDLN handle;
    uint8_t led_count;
    DLN_RESULT result = DlnLedGetCount(handle, &led_count);*/

    DlnConnect("localhost", DLN_DEFAULT_SERVER_PORT);

    //DlnCloseAllHandles();
    uint32_t deviceCount;
    DLN_RESULT result = DlnGetDeviceCount(&deviceCount);
    if (DLN_SUCCEEDED(result))
    {
        printf("Got DLN device count: %d\n", deviceCount);
        for (uint32_t i = 0; i < deviceCount; i++)
        {
            HDLN handle;
            result = DlnOpenDevice(i, &handle);
            if (DLN_SUCCEEDED(result))
            {
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
            }
            DlnCloseHandle(handle);
        }
    }

    DlnDisconnectAll();

    return 0;
}
