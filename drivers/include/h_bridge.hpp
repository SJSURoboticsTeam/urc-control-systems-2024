#pragma once
#include <array>
#include <libhal/pointers.hpp>
#include <libhal/motor.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
namespace sjsu::drivers {

class h_bridge : public hal::motor
{
public:
  struct phase
  {
    hal::v5::optional_ptr<hal::pwm16_channel> p_high;
    hal::v5::optional_ptr<hal::output_pin> p_low;
  };
  explicit h_bridge(phase p_pin_a, phase p_pin_b)
    : m_pin_a_high(p_pin_a.p_high)
    , m_pin_b_high(p_pin_b.p_high)
    , m_pin_a_low(p_pin_a.p_low)
    , m_pin_b_low(p_pin_b.p_low)
  {
  }

private:
  hal::v5::optional_ptr<hal::pwm16_channel> m_pin_a_high;
  hal::v5::optional_ptr<hal::pwm16_channel> m_pin_b_high;
  hal::v5::optional_ptr<hal::output_pin> m_pin_a_low;
  hal::v5::optional_ptr<hal::output_pin> m_pin_b_low;

  void driver_power(float p_power);
};
}  // namespace sjsu::drivers
