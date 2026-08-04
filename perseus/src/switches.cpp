#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <libhal/input_pin.hpp>

#include <switches.hpp>
#include <resource_list.hpp>


using sec = float;

namespace sjsu::perseus {

switches_bldc::switches_bldc(
    hal::v5::strong_ptr<hal::input_pin> p_s1, 
    hal::v5::strong_ptr<hal::input_pin> p_s2, 
    hal::v5::strong_ptr<hal::input_pin> p_s3, 
    hal::v5::strong_ptr<hal::input_pin> p_s4, 
    hal::v5::strong_ptr<hal::input_pin> p_s5, 
    hal::v5::strong_ptr<hal::input_pin> p_s6
  ) : 
    m_s1(p_s1), m_s2(p_s2), m_s3(p_s3), m_s4(p_s4), m_s5(p_s5), m_s6(p_s6)
{
  m_switch_value = (static_cast<hal::u8>(m_s1->level()) << 5)
    | (static_cast<hal::u8>(m_s2->level()) << 4)
    | (static_cast<hal::u8>(m_s3->level()) << 3)
    | (static_cast<hal::u8>(m_s4->level()) << 2)
    | (static_cast<hal::u8>(m_s5->level()) << 1)
    | (static_cast<hal::u8>(m_s6->level()));
};

// switch value 
hal::u8 switches_bldc::read_switch_value() {
  m_switch_value = (static_cast<hal::u8>(m_s1->level()) << 5)
  | (static_cast<hal::u8>(m_s2->level()) << 4)
  | (static_cast<hal::u8>(m_s3->level()) << 3)
  | (static_cast<hal::u8>(m_s4->level()) << 2)
  | (static_cast<hal::u8>(m_s5->level()) << 1)
  | (static_cast<hal::u8>(m_s6->level())); 
  return m_switch_value;
} 


} // namespace sjsu::perseus