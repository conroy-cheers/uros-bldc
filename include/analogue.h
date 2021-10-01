#include "stm32g4xx_hal.h"

constexpr float _analogue_temp_rmean_weight = 0.8;
static int32_t _analogue_temp_rmean;

static int32_t read_vref() {
    return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF),
                                             LL_ADC_RESOLUTION_12B));
}

static int32_t read_temp_sensor(int32_t vref) {
    return (__LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP),
                                      LL_ADC_RESOLUTION_12B));
}

static int32_t read_temp_sensor() {
    int32_t vref = read_vref();
    return read_temp_sensor(vref);
}

static int32_t temp_filter(int32_t temp) {
    // outlier rejection
    if (temp < 0.6 * _analogue_temp_rmean ||
        temp > 1.6 * _analogue_temp_rmean) {
        return _analogue_temp_rmean;
    }

    _analogue_temp_rmean = _analogue_temp_rmean * _analogue_temp_rmean_weight +
                           temp * (1 - _analogue_temp_rmean_weight);
    return _analogue_temp_rmean;
}

static void analogue_setup() {
    analogReadResolution(12);

    _analogue_temp_rmean = read_temp_sensor();
}
