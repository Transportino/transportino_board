#include "hardware/temperature.h"
#include <hardware/adc.h>

terror temp_init(temp_sensor* temp_sensor)
{
    /* Initialize hardware AD converter, enable onboard temperature sensor and
     *   select its channel (do this once for efficiency, but beware that this
     *   is a global operation). */
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    temp_sensor->temperature = 0.0;
    return NULL_ERROR;
}

terror temp_read(temp_sensor* temp_sensor)
{
    // For more info see: https://github.com/raspberrypi/pico-examples/blob/master/adc/onboard_temperature/onboard_temperature.c
    
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);
    float adc = (float)adc_read() * conversionFactor;
    temp_sensor->temperature = 27.0f - (adc - 0.706f) / 0.001721f;
    return NULL_ERROR;
}