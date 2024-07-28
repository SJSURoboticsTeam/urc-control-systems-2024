#pragma once

#include <libhal/input_pin.hpp>
#include <libhal/adc.hpp>
#include <cmath>

#include "adc_sensor.hpp"

namespace sjsu::science{

/// @brief Implementation driver for MQ4 methane sensor
class mq4_methane_sensor : generic_adc_sensor{
public:

    /// @brief Builds MQ4 sensor driver object.
    /// @param adc_data libhal adc pin that has been initialized 
    /// @param digital_detector Digital pin that goes HIGH when methane is detected. 
    mq4_methane_sensor(hal::adc& adc_data) : generic_adc_sensor(adc_data) {};

    /// @brief Returns read value from sensor in parts per million (PPM).
    /// @return The amount of methane near the sensor in PPM.
    virtual float get_parsed_data() override{
        float raw_adc_value = read_raw_adc();

        float ppm_value = static_cast<float>(std::pow(1000 * raw_adc_value, -2.95));
        return ppm_value;
    }
};
}