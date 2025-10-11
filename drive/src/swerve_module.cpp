#pragma once
#include "../include/swerve_module.hpp"
#include <libhal/units.hpp>

namespace sjsu::drive {
swerve_module::swerve_module(hal::actuator::rmd_mc_x_v2& p_steer_motor,
                             hal::actuator::rmd_mc_x_v2& p_propulsion_motor,
                             swerve_module_settings p_settings)
  : m_steer_motor(p_steer_motor)
  , m_propulsion_motor(p_propulsion_motor)
  , settings(p_settings)
{
}
void swerve_module::stop()
{
  m_steer_motor.velocity_control(0);
  m_propulsion_motor.velocity_control(0);
}

void swerve_module::set_target_state(swerve_module_state const& p_target_state)
{
  m_target_state = p_target_state;
  if (can_reach_state(m_target_state)) {
    m_steer_motor.position_control(m_target_state.steer_angle, 50);
    m_propulsion_motor.velocity_control(m_target_state.propulsion_velocity);
  }
}

bool swerve_module::can_reach_state(swerve_module_state const& p_state)
{
  if ((p_state.propulsion_velocity < settings.max_speed) &&
      (p_state.steer_angle > settings.min_angle) &&
      (p_state.steer_angle < settings.max_angle)) {
    return true;
  }
  return false;
}

swerve_module_state swerve_module::get_actual_state_cache() const
{
  return m_actual_state_cache;
}

swerve_module_state swerve_module::refresh_actual_state_cache()
{
  m_actual_state_cache.steer_angle = m_steer_motor.feedback().angle();
  m_actual_state_cache.propulsion_velocity =
    m_propulsion_motor.feedback().speed();
  return m_actual_state_cache;
}

}  // namespace sjsu::drive
