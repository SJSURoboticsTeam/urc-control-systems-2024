#include <libhal/units.hpp>
#include <velocity_motor_mock.hpp>

namespace sjsu::drivers {
velocity_motor_mock::velocity_motor_mock(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::rpm p_max_speed,
  float p_max_acceleration)
  : m_clock(p_clock)
  , m_max_speed(p_max_speed)
  , m_max_acceleration(p_max_acceleration)
{
  m_sim_time = m_clock->uptime();
}

hal::rpm velocity_motor_mock::update_sim()
{
  hal::u64 time = m_clock->uptime();
  hal::rpm target_vel = m_enabled ? m_target_velocity : 0;
  hal::degrees m_max_velocity_accelerated =
    (time - m_sim_time) / m_clock->frequency() * (m_max_acceleration);
  if (m_max_velocity_accelerated > std::abs(target_vel - m_sim_velocity)) {
    m_sim_velocity = target_vel;
  } else {
    if (target_vel < m_sim_velocity) {
      m_sim_velocity += m_max_velocity_accelerated;
    } else {
      m_sim_velocity -= m_max_velocity_accelerated;
    }
  }
  m_sim_time = time;
  return m_sim_velocity;
}

void velocity_motor_mock::driver_enable(bool p_state)
{
  update_sim();
  m_enabled = p_state;
}
void velocity_motor_mock::driver_drive(hal::rpm p_velocity)
{
  update_sim();
  m_target_velocity = p_velocity;
}
hal::velocity_motor::status_t velocity_motor_mock::driver_status()
{
  return { .velocity = update_sim() };
}
hal::velocity_motor::range_t velocity_motor_mock::driver_velocity_range()
{
  return { .max = m_max_speed };
}

}  // namespace sjsu::drivers
