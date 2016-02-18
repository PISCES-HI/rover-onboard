#include <iostream>

#include "dln/dln_generic.h"
#include "dln/dln_adc.h"

#include "analog.h"

void init_analog(HDLN handle) {
    DlnAdcSetResolution(handle, 0, DLN_ADC_RESOLUTION_10BIT);

    // Enable channel 0 with port 0
    DlnAdcChannelEnable(handle, 0, 0);

    // Enable port 0
    uint16_t conflict;
    DLN_RESULT result = DlnAdcEnable(handle, 0, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }
}

void cleanup_analog(HDLN handle) {
    DlnAdcDisable(handle, 0);
    DlnAdcChannelDisable(handle, 0, 0);
}
