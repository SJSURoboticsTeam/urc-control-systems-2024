#include <propulsion_controller_rmd_x7.hpp>

namespace sjsu::drive {
propulsion_controller_rmd_x7::propulsion_controller_rmd_x7(
  hal::v5::strong_ptr<hal::actuator::rmd_drc_v2> p_motor)
  : m_motor(p_motor)
{
}
void propulsion_controller_rmd_x7::stop()
{
  set_target_velocity(0);
}
void propulsion_controller_rmd_x7::set_target_velocity(hal::rpm p_velocity)
{
  m_target_velocity = p_velocity;
  m_motor->velocity_control(p_velocity);
}
hal::rpm propulsion_controller_rmd_x7::get_target_velocity() {
  return m_target_velocity;
}
hal::rpm propulsion_controller_rmd_x7::get_actual_velocity() {
  return m_motor->feedback().speed();
}
}  // namespace sjsu::drive
