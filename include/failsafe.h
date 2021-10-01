#include <Arduino.h>

constexpr int32_t failsafe_temp_lower = 60;
constexpr int32_t failsafe_temp_upper = 75;

static int32_t failsafe_temp = failsafe_temp_upper;

/**
 * @brief Gets the temperature failsafe status, and applies hysteresis.
 * 
 * @param temp Current temperature, in degrees Celsius
 * @return true where the provided temperature is safe
 * @return false where the provided temperature is not safe
 */
bool is_safe(int32_t temp) {
    if (temp < failsafe_temp) {
        failsafe_temp = failsafe_temp_upper;
        return true;
    } else {
        failsafe_temp = failsafe_temp_lower;
        return false;
    }
}
