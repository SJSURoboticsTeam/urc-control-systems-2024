#include <h_bridge.hpp>

namespace sjsu::drivers {
void h_bridge::driver_power(float p_power)
{
  if (p_power < 0) {
    m_pin_b_low.duty_cycle(0);
    m_pin_a_high.level(false);
    m_pin_b_high.level(true);
    m_pin_a_low.duty_cycle((-1) * p_power * 0xFFFF);
  } else {
    m_pin_a_low.duty_cycle(0);
    m_pin_b_high.level(false);
    m_pin_b_low.duty_cycle(p_power * 0xFFFF);
    m_pin_a_high.level(true);
  }
}

}  // namespace sjsu::drivers
