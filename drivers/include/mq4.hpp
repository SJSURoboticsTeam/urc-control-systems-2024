#pragma once

#include <libhal/input_pin.hpp>
#include <libhal/adc.hpp>
#include <cmath>

namespace sjsu::drivers{

/// @brief A Generic ADC Driver. This is designed to be a higher level interface for analog devices.
class mq4 {
public:
    /// @brief Builds MQ4 sensor driver object.
    /// @param adc_data libhal adc pin that has been initialized 
    /// @param digital_detector Digital pin that goes HIGH when methane is detected. 
    mq4(hal::adc& p_adc);


    /// @brief Reads the raw voltage value from the ADC pin. 
    /// Does no unit conversion outside of converting the analog signal to a digital signal.
    /// @return The voltage value on the analog data pin.
    float read_raw_adc();

    //// @brief Returns read value from sensor in parts per million (PPM).
    /// @return The amount of methane near the sensor in PPM.
    float get_parsed_data();
        
private:
    hal::adc& m_adc;
};
} //namespace drivers