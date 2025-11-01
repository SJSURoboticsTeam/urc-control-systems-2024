#include "../include/swerve_module.hpp"
#include "resource_list.hpp"
#include <cmath>
#include <cstdlib>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;

namespace sjsu::drive {
swerve_module::swerve_module(
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_steer_motor,
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_propulsion_motor,
  hal::v5::strong_ptr<hal::input_pin> p_limit_switch,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  swerve_module_settings p_settings)
  : settings(p_settings)
  , m_steer_motor(p_steer_motor)
  , m_propulsion_motor(p_propulsion_motor)
  , m_limit_switch(p_limit_switch)
  , m_clock(p_clock)
{
  // TODO: verify settings were initalized
}

void swerve_module::stop()
{
  m_steer_motor->velocity_control(0);
  m_propulsion_motor->velocity_control(0);
}

void swerve_module::set_target_state(swerve_module_state const& p_target_state)
{
  if (m_steer_offset == NAN) {
    throw hal::resource_unavailable_try_again(this);
  }
  m_target_state = p_target_state;
  if (!can_reach_state(m_target_state)) {
    throw hal::argument_out_of_domain(this);
  }
  // auto console = resources::console();
  // m_steer_motor->feedback_request(
  //   hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  // hal::print<128>(
  //   *console, "Cur angle: %f\n", m_steer_motor->feedback().angle());
  // hal::print<128>(*console,
  //                 "Target angle: %f\n",
  //                 m_target_state.steer_angle + m_steer_offset);
  // m_steer_motor->position_control(m_steer_motor->feedback().angle(),
  //                                 1);
  m_steer_motor->position_control(m_target_state.steer_angle + m_steer_offset,
                                  120);
  // m_propulsion_motor->velocity_control(m_target_state.propulsion_velocity);
}

bool swerve_module::can_reach_state(swerve_module_state const& p_state) const
{
  return ((p_state.propulsion_velocity <= std::abs(settings.max_speed)) &&
          (p_state.steer_angle >= settings.min_angle) &&
          (p_state.steer_angle <= settings.max_angle));
}

swerve_module_state swerve_module::get_actual_state_cache() const
{
  return m_actual_state_cache;
}

swerve_module_state swerve_module::refresh_actual_state_cache()
{
  m_steer_motor->feedback_request(
    hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  m_actual_state_cache.steer_angle =
    m_steer_motor->feedback().angle() - m_steer_offset;
  m_actual_state_cache.propulsion_velocity =
    m_propulsion_motor->feedback().speed();
  return m_actual_state_cache;
}

void swerve_module::hard_home()
{
  // auto console = resources::console();
  // hal::print(*console, "starting hard home changed\n");
  m_steer_motor->feedback_request(
    hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  m_steer_motor->velocity_control(0);  // dummy message
  // hal::print<128>(*console, "Start level: %d\n", m_limit_switch->level());
  [[__maybe_unused__]] float start_angle = m_steer_motor->feedback().angle();
  if (settings.home_clockwise) {
    m_steer_motor->velocity_control(-1);
  } else {
    m_steer_motor->velocity_control(1);
  }
  while (!m_limit_switch->level()) {
    hal::delay(*m_clock, 250ms);//250ms seams safe refresh time
  }
  m_steer_motor->velocity_control(0);  // stops
  // hal::print<128>(*console, "Final level: %d\n", m_limit_switch->level());
  
  m_steer_motor->feedback_request(
    hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  float stop_angle = m_steer_motor->feedback().angle();

  // hal::print<128>(*console, "Stopped angle: %f\n", stop_angle);
  m_steer_offset = stop_angle - settings.limit_switch_position;
  // hal::print<128>(
  //   *console, "fin position: %f\n", refresh_actual_state_cache().steer_angle);
  set_target_state(swerve_module_state(0, 0));
}

}  // namespace sjsu::drive
