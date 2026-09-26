#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <steer_controller_perseus.hpp>

namespace sjsu::drive {

using namespace std::chrono_literals;

steer_controller_perseus::steer_controller_perseus(
  hal::v5::strong_ptr<drivers::perseus_bldc> p_perseus,
  hal::v5::strong_ptr<hal::steady_clock> p_clock)
  : m_perseus(p_perseus)
  , m_clock(p_clock)
{
}

void steer_controller_perseus::stop()
{
  m_perseus->kill_power();
}

void steer_controller_perseus::hard_home()
{
  m_perseus->home();
  while (m_perseus->is_homing()) {
    hal::delay(*m_clock, 10ms);
  }
  m_is_homed = true;
}

void steer_controller_perseus::home()
{
  m_perseus->home();
}

void steer_controller_perseus::home_periodic()
{
  if (!m_perseus->is_homing()) {
    // not done homing and no homing in progress -> finish
    m_is_homed = true;
  } else {
    // not done homing and homing in progress -> OK
  }
}

void steer_controller_perseus::stop_home()
{
  // effectively cancels homing by telling motor to stop moving
  m_perseus->set_target_velocity(0);
}

bool steer_controller_perseus::is_homing()
{
  return m_perseus->is_homing();
}

bool steer_controller_perseus::is_homed()
{
  return m_is_homed;
}

void steer_controller_perseus::set_target_position(
  hal::degrees p_target_position)
{
  if (!m_is_homed) {
    throw hal::exception(std::errc::operation_not_permitted, &m_perseus);
  }
  m_target_position = p_target_position;
  m_perseus->set_target_position(p_target_position);
}

hal::degrees steer_controller_perseus::get_target_postion()
{
  if (!m_is_homed) {
    throw hal::exception(std::errc::operation_not_permitted, &m_perseus);
  }
  // TODO: make get target position on perseus when implemented
  return m_target_position;
}

hal::degrees steer_controller_perseus::get_actual_postion()
{
  if (!m_is_homed) {
    throw hal::exception(std::errc::operation_not_permitted, &m_perseus);
  }
  return m_perseus->get_position();
}
}  // namespace sjsu::drive
