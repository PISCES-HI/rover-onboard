#include <iostream>

#include "dln/dln_generic.h"
#include "dln/dln_adc.h"

#include "analog.h"

void init_analog(HDLN handle) {
    DlnAdcSetResolution(handle, 0, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 1, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 2, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 3, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 4, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 5, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 6, DLN_ADC_RESOLUTION_10BIT);
    DlnAdcSetResolution(handle, 7, DLN_ADC_RESOLUTION_10BIT);

    // Enable port 0 with channel 0
    DlnAdcChannelEnable(handle, 0, 0);
    DlnAdcChannelEnable(handle, 1, 0);
    DlnAdcChannelEnable(handle, 2, 0);
    DlnAdcChannelEnable(handle, 3, 0);
    DlnAdcChannelEnable(handle, 4, 0);
    DlnAdcChannelEnable(handle, 5, 0);
    DlnAdcChannelEnable(handle, 6, 0);
    DlnAdcChannelEnable(handle, 7, 0);
    DlnAdcChannelEnable(handle, 8, 0);

    // Enable port 0
    uint16_t conflict;
    DLN_RESULT result = DlnAdcEnable(handle, 0, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 1
    result = DlnAdcEnable(handle, 1, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 2
    result = DlnAdcEnable(handle, 2, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 3
    result = DlnAdcEnable(handle, 3, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 4
    result = DlnAdcEnable(handle, 4, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 5
    result = DlnAdcEnable(handle, 5, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 6
    result = DlnAdcEnable(handle, 6, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }

    // Enable port 7
    result = DlnAdcEnable(handle, 7, &conflict);
    if (DLN_FAILED(result)) {
        std::cout << "DlnAdcEnable() error " << result << std::endl;
        return;
    }
}

void cleanup_analog(HDLN handle) {
    DlnAdcDisable(handle, 0);
    DlnAdcDisable(handle, 1);
    DlnAdcDisable(handle, 2);
    DlnAdcDisable(handle, 3);
    DlnAdcDisable(handle, 4);
    DlnAdcDisable(handle, 5);
    DlnAdcDisable(handle, 6);
    DlnAdcDisable(handle, 7);
    DlnAdcChannelDisable(handle, 0, 0);
    DlnAdcChannelDisable(handle, 0, 1);
    DlnAdcChannelDisable(handle, 0, 2);
    DlnAdcChannelDisable(handle, 0, 3);
    DlnAdcChannelDisable(handle, 0, 4);
    DlnAdcChannelDisable(handle, 0, 5);
    DlnAdcChannelDisable(handle, 0, 6);
    DlnAdcChannelDisable(handle, 0, 7);
}
