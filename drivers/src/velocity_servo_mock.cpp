#include <libhal/units.hpp>
#include <velocity_servo_mock.hpp>

namespace sjsu::drivers {
velocity_servo_mock::velocity_servo_mock(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::rpm p_max_speed,
  position_range_t p_position_range,
  hal::degrees p_inital_position)
  : m_sim_position(p_inital_position)
  , m_clock(p_clock)
  , m_max_speed(p_max_speed)
  , m_target_position(p_inital_position)
  , m_postion_range(p_position_range)
{
  m_sim_time = m_clock->uptime();
}

hal::degrees velocity_servo_mock::update_sim()
{
  hal::u64 time = m_clock->uptime();
  if (not m_enabled) {
    m_sim_time = time;
    return m_sim_position;
  }
  hal::degrees max_dist_traveled =
    (time - m_sim_time) / m_clock->frequency() * (m_max_speed / 60.0f * 360.0f);
  if (max_dist_traveled > std::abs(m_target_position - m_sim_position)) {
    m_sim_position = m_target_position;
  } else {
    if (m_target_position < m_sim_position) {
      m_sim_position += max_dist_traveled;
    } else {
      m_sim_position -= max_dist_traveled;
    }
  }
  m_sim_time = time;
  return m_sim_position;
}
void velocity_servo_mock::driver_enable(bool p_state)
{
  update_sim();
  m_enabled = p_state;
}

void velocity_servo_mock::driver_position(hal::degrees p_target_position)
{
  update_sim();
  m_target_position = p_target_position;
}
hal::velocity_servo::position_range_t
velocity_servo_mock::driver_position_range()
{
  return m_postion_range;
}

hal::degrees velocity_servo_mock::driver_get_position()
{
  return update_sim();
}
bool velocity_servo_mock::driver_is_moving()
{
  return driver_get_position() != m_target_position;
}
void velocity_servo_mock::driver_configure(settings const& p_settings)
{
  m_max_speed = p_settings.velocity;
}
hal::velocity_servo::status_t velocity_servo_mock::driver_status()
{
  if (!driver_is_moving()) {
    return { .velocity = 0 };
  }
  if (m_sim_position < m_target_position) {
    return { .velocity = m_max_speed };
  } else {
    return { .velocity = -m_max_speed };
  }
}
hal::velocity_servo::range_t velocity_servo_mock::driver_velocity_range()
{
  return { .max = m_max_speed };
}

};  // namespace sjsu::drivers
