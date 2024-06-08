#pragma once

#include <libhal/input_pin.hpp>
#include <libhal/adc.hpp>
#include <cmath>

namespace sjsu::science{

/// @brief A Generic ADC Driver. This is designed to be a higher level interface for analog devices.
class generic_adc_sensor {
public:
    /// @brief Builds driver object
    /// @param adc_data libhal adc pin that has been initialized 
    /// @param digital_detector Optional digital pin for device that is to goes high when a signal is detected
    generic_adc_sensor(hal::adc& adc_data) : adc_data_(adc_data) {}


    /// @brief Reads the raw voltage value from the ADC pin. 
    /// Does no unit conversion outside of converting the analog signal to a digital signal.
    /// @return The voltage value on the analog data pin.
    hal::result<float> read_raw_adc() {
        float raw_ratio_average = 0;
        int read_count = 10;
        for (int i = 0; i < read_count; i++)
        {
            raw_ratio_average += HAL_CHECK(adc_data_.read()).sample;
        }
        
        raw_ratio_average /= read_count;

        return raw_ratio_average;
    }

    /// @brief Processes data into usable units.
    /// Implementations of this method should include a mathmatical conversion formula that goes from raw voltage
    /// to desired units or units specified in device's datasheet. 
    /// @return Converted value from voltage to desired units.
    virtual hal::result<float> get_parsed_data() = 0;
        
protected:
    hal::adc& adc_data_;
};
} //namespace science