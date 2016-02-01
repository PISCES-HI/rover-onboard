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

    // Initialize the i2c master
    init_i2c(handle);

    // Instantiate the rover controller
    RoverControl rover(handle);

    while (true) {
        rover.update();
    }

    DlnCloseHandle(handle);

    DlnDisconnectAll();

    return 0;
}
