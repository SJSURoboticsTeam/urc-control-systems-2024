#include <array>
#include <libhal-util/steady_clock.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

using sec = float;

namespace sjsu::perseus {

switches_bldc::switches_bldc(hal::v5::strong_ptr<hal::input_pin> p_s1,
                             hal::v5::strong_ptr<hal::input_pin> p_s2,
                             hal::v5::strong_ptr<hal::input_pin> p_s3,
                             hal::v5::strong_ptr<hal::input_pin> p_s4,
                             hal::v5::strong_ptr<hal::input_pin> p_s5,
                             hal::v5::strong_ptr<hal::input_pin> p_s6)
  :  // m_s1(p_s1), m_s2(p_s2), m_s3(p_s3), m_s4(p_s4), m_s5(p_s5), m_s6(p_s6)
  m_switch_array({ p_s1, p_s2, p_s3, p_s4, p_s5, p_s6 }) {};

// switch value
hal::u8 switches_bldc::read_switch_value()
{
  hal::u8 switch_value = 0x000000000000;
  for (size_t i = 0; i < m_switch_array.size(); i++) {
    switch_value |= static_cast<hal::u8>(m_switch_array[i]->level())
                    << (m_switch_array.size() - 1 - i);
  }
  return switch_value;
}

}  // namespace sjsu::perseus
