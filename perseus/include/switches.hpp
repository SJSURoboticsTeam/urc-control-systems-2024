#pragma once
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <libhal/input_pin.hpp>

#include <resource_list.hpp>


using sec = float;

namespace sjsu::perseus {

class switches_bldc
{

public:
    switches_bldc(hal::v5::strong_ptr<hal::input_pin> p_s1, 
                        hal::v5::strong_ptr<hal::input_pin> p_s2, 
                        hal::v5::strong_ptr<hal::input_pin> p_s3, 
                        hal::v5::strong_ptr<hal::input_pin> p_s4, 
                        hal::v5::strong_ptr<hal::input_pin> p_s5, 
                        hal::v5::strong_ptr<hal::input_pin> p_s6
                    ); 
    /**
     * @brief Reads the number the switches represent.  
     * @return The current number the switches are at (on = 1, off = 0, binary rules)
    */
    hal::u8 read_switch_value(); 

private: 
    hal::v5::strong_ptr<hal::input_pin> m_s1;
    hal::v5::strong_ptr<hal::input_pin> m_s2;
    hal::v5::strong_ptr<hal::input_pin> m_s3;
    hal::v5::strong_ptr<hal::input_pin> m_s4;
    hal::v5::strong_ptr<hal::input_pin> m_s5;
    hal::v5::strong_ptr<hal::input_pin> m_s6;
    hal::u8 m_switch_value; 
};

} // namespace sjsu::perseus