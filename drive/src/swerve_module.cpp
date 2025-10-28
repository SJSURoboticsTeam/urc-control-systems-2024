#include "../include/swerve_module.hpp"
#include <cstdlib>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/pointers.hpp>

using namespace std::chrono_literals;
using namespace hal::literals;

namespace sjsu::drive {
swerve_module::swerve_module(hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_steer_motor,
                             hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_propulsion_motor,
                             hal::v5::strong_ptr<hal::input_pin> p_limit_switch,
                             hal::v5::strong_ptr<hal::steady_clock> p_clock,
                             hal::v5::strong_ptr<hal::serial> p_console,
                             swerve_module_settings p_settings)
  : settings(p_settings)
  , m_steer_motor(p_steer_motor)
  , m_propulsion_motor(p_propulsion_motor)
  , m_limit_switch(p_limit_switch)
  , m_clock(p_clock)
  , m_console(p_console)
{
}

void swerve_module::stop()
{
  m_steer_motor->velocity_control(0);
  m_propulsion_motor->velocity_control(0);
}

void swerve_module::set_target_state(swerve_module_state const& p_target_state)
{
  m_target_state = p_target_state;
  if (can_reach_state(m_target_state)) {
    m_steer_motor->position_control(m_target_state.steer_angle, 50);
    m_propulsion_motor->velocity_control(m_target_state.propulsion_velocity);
  }
}

bool swerve_module::can_reach_state(swerve_module_state const& p_state)
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
  m_actual_state_cache.steer_angle = m_steer_motor->feedback().angle();
  m_actual_state_cache.propulsion_velocity =
    m_propulsion_motor->feedback().speed();
  return m_actual_state_cache;
}

void swerve_module::home(){
    m_steer_motor->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    m_steer_motor->velocity_control(0); // dummy message
    hal::print<128>(*m_console, "Start level: %d\n" , m_limit_switch->level());
    [[__maybe_unused__]] float start_angle = m_steer_motor->feedback().angle();
    while (!m_limit_switch->level()){
      hal::print<128>(*m_console, "level: %d\n" , m_limit_switch->level());
      if (settings.reversed) {
        m_steer_motor->velocity_control(-1);
        hal::delay(*m_clock,5ms);
      } else {
        m_steer_motor->velocity_control(1);
        hal::delay(*m_clock, 5ms);
      }
    }
    hal::print<128>(*m_console, "Final level: %d\n" , m_limit_switch->level());
    m_steer_motor->velocity_control(0);  // stops
    hal::delay(*m_clock, 1000ms);
    
    m_steer_motor->feedback_request(hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float stop_angle = m_steer_motor->feedback().angle();

    m_steer_motor->velocity_control(0);  // stops
    hal::delay(*m_clock, 1000ms);
    
    if(settings.reversed){
      m_steer_motor->position_control(stop_angle + 90, 1);
    }else{
      m_steer_motor->position_control(stop_angle - 90, 1);
    }
    stop_angle = m_steer_motor->feedback().angle();

    hal::print<128>(*m_console, "Stopped angle: %f\n", stop_angle);
    settings.steer_offset = stop_angle;
    is_homed = true;
}

}  // namespace sjsu::drive
