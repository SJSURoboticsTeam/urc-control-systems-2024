#include <clamped_servo.hpp>

namespace sjsu::drivers {

clamped_servo::clamped_servo(hal::strong_ptr<hal::servo> p_servo,
                             hal::basic_servo::position_range_t p_range)
  : m_servo{ p_servo }
  , m_range{ p_range }
{
}

void clamped_servo::driver_position(hal::degrees p_position)
{
  auto clamped_pos = std::clamp(p_position, m_range.min, m_range.max);
  m_servo->position(clamped_pos);
}

}  // namespace sjsu::drivers
