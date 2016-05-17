#include <cmath>

#include "thermistor.h"

double get_thermistor_temp(uint16_t adc_reading) {
    const double a = 0.0007170311911;
    const double b = 0.0002176343272;
    const double c = 0.00000008609219282;
    
    const double vcc = 4.87;
    const double R2 = 99900;

    double v2 = (((double)adc_reading)/1024.0)*vcc;

    double temp = log((vcc-v2)*(R2/v2));
    temp = 1.0 / (a + (b + (c * temp * temp ))*temp);

    return temp - 273.15;
}
