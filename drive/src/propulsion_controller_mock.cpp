#include <propulsion_controller_mock.hpp>

namespace sjsu::drive {
propulsion_controller_mock::propulsion_controller_mock(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::rpm p_max_speed,
  float p_max_acceleration) : m_motor(p_clock, p_max_speed, p_max_acceleration){
  }
void propulsion_controller_mock::stop() {
  set_target_velocity(0);
}
void propulsion_controller_mock::set_target_velocity(hal::rpm p_velocity) {
  m_target_velocity = p_velocity;
  m_motor.drive(p_velocity);
}
hal::rpm propulsion_controller_mock::get_target_velocity() {
  return m_target_velocity;
}
hal::rpm propulsion_controller_mock::get_actual_velocity() {
 return m_motor.status().velocity;
}
}  // namespace sjsu::drive
