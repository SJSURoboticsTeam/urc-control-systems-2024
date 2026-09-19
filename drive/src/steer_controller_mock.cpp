
#include <libhal/units.hpp>
#include <steer_controller_mock.hpp>
namespace sjsu::drive {

steer_controller_mock::steer_controller_mock(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::rpm p_max_speed,
  hal::degrees p_inital_position)
  : m_servo(p_clock,
            p_max_speed,
            { .min = -std::numeric_limits<float>::infinity(),
              .max = std::numeric_limits<float>::infinity() },
            p_inital_position)
{
}

void steer_controller_mock::stop() {
  set_target_position(get_actual_postion());
}
void steer_controller_mock::hard_home()
{
  m_is_homed = true;
}
void steer_controller_mock::home()
{
  m_is_homed = true;
}
void steer_controller_mock::home_periodic()
{
  return;
}
bool steer_controller_mock::is_homing()
{
  return false;
}
bool steer_controller_mock::is_homed()
{
  return m_is_homed;
}
void steer_controller_mock::set_target_position(hal::degrees p_target_position)
{
  m_target_position = p_target_position;
}
hal::degrees steer_controller_mock::get_target_postion() {
  return m_target_position;
}
hal::degrees steer_controller_mock::get_actual_postion() {
  return m_servo.position();
}
}  // namespace sjsu::drive
