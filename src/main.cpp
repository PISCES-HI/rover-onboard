#include <iostream>

#include "dln/dln.h"
#include "dln/dln_generic.h"
#include "dln/dln_gpio.h"

#include "analog.h"
#include "i2c.h"
#include "PwmDriver.h"
#include "RoverControl.h"

#include "Serial.h"
#include "GpsDriver.h"

int main() {
    DLN_RESULT result;

    DlnConnect("localhost", DLN_DEFAULT_SERVER_PORT);

    // Check device count
    uint32_t device_count;
    result = DlnGetDeviceCount(&device_count);
    if (!DLN_SUCCEEDED(result)) {
        std::cout << "Failed to get DLN device count" << std::endl;
        return -1;
    }
    if (device_count != 1) {
        std::cout << "There should be one and only one DLN device connected. There are "
                  << device_count << std::endl;
        return -1;
    }

    // Try to open our device
    HDLN handle;
    result = DlnOpenDevice(0, &handle);
    if (!DLN_SUCCEEDED(result)) {
        std::cout << "Failed to open DLN device" << std::endl;
        return -1;
    }

    // Print out some info about our DLN device
    DLN_VERSION version;
    uint32_t sn, id;
    DlnGetVersion(handle, &version);
    DlnGetDeviceSn(handle, &sn);
    DlnGetDeviceId(handle, &id);
    std::cout << "Opened DLN device: " << sn << "\t" << id << std::endl;

    // Initialize all the things
    init_i2c(handle);
    init_analog(handle);

    // Instantiate the rover controller
    RoverControl rover(handle);

    while (true) {
        rover.update();
    }

    cleanup_analog(handle);

    DlnCloseHandle(handle);

    DlnDisconnectAll();

    return 0;
}
