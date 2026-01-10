#include "../include/swerve_module.hpp"
#include <resource_list.hpp>
#include <cmath>
#include <cstdlib>
#include <drivetrain_math.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <swerve_module.hpp>

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
  , m_tolerance_last_changed()
{
  // TODO: verify settings were initalized
}

void swerve_module::stop()
{
  m_steer_motor->velocity_control(0);
  m_propulsion_motor->velocity_control(0);
}

bool swerve_module::stopped() const
{
  return m_target_state.propulsion_velocity == 0 &&
         std::abs(m_actual_state_cache.steer_angle) <=
           settings.velocity_tolerance;
}

void swerve_module::set_target_state(swerve_module_state const& p_target_state)
{
  auto console = resources::console();
  hal::print<128>(*console,
                  "set_target_state:%f,%f\n",
                  p_target_state.steer_angle,
                  p_target_state.propulsion_velocity);
  if (m_steer_offset == NAN) {
    throw hal::resource_unavailable_try_again(this);
  }
  if (!can_reach_state(m_target_state)) {
    throw hal::argument_out_of_domain(this);
  }
  m_target_state = p_target_state;
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
                                  30);
  hal::rpm velocity = m_target_state.propulsion_velocity * settings.mps_to_rpm;
  if (settings.drive_forward_clockwise) {
    velocity *= -1;
  }
  m_propulsion_motor->velocity_control(velocity);
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
    m_propulsion_motor->feedback().speed() / settings.mps_to_rpm;
  if (settings.drive_forward_clockwise) {
    m_actual_state_cache.propulsion_velocity *= -1;
  }
  return m_actual_state_cache;
}

swerve_module_state swerve_module::get_target_state() const
{
  return m_target_state;
}

void swerve_module::update_tolerance_debouncer()
{
  // 1. the target angle is always within the absolute min/max for the angle
  // 2. if the actual angle is not within tolerance of the target, then we
  // state angle is out of tolerance
  // 3. if actual angle is within tolerance of the target, and we know target
  // is always within absolute min/max, then we know actual angle is within
  // tolerance of the absolute min/max
  // 4. therefore, we do not actually need to check absolute min/max bounds
  bool angle_out_of_tolerance =
    fabs(m_actual_state_cache.steer_angle - m_target_state.steer_angle) >=
    settings.position_tolerance;

  // similar reasoning as the angle out of tolerance, we do not need to check
  // max speed
  bool velocity_out_of_tolerance =
    fabs(m_actual_state_cache.propulsion_velocity -
         m_target_state.propulsion_velocity) >= settings.velocity_tolerance;

  // true if out of tolerance
  bool current_state = angle_out_of_tolerance || velocity_out_of_tolerance;
  auto current_time = get_clock_time(*m_clock);

  // if deviated from stable
  if (m_stable_tolerance_state != current_state) {
    m_tolerance_last_changed = current_time;
  }

  sec dt = hal_time_duration_to_sec(current_time - m_tolerance_last_changed);
  // if stayed stable for timeout time
  if (dt > settings.tolerance_timeout) {
    m_stable_tolerance_state = current_state;
  }
}

bool swerve_module::tolerance_timed_out() const
{
  return m_stable_tolerance_state;
}

void swerve_module::hard_home()
{
  auto console = resources::console();
  hal::print(*console, "starting hard home changed\n");
  m_steer_motor->feedback_request(
    hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  m_steer_motor->velocity_control(0);  // dummy message
  hal::print<128>(*console, "Start level: %d\n", m_limit_switch->level());
  [[__maybe_unused__]] float start_angle = m_steer_motor->feedback().angle();
  if (settings.home_clockwise) {
    m_steer_motor->velocity_control(-1);
  } else {
    m_steer_motor->velocity_control(1);
  }
  // while (m_limit_switch->level()) {
  //   hal::delay(*m_clock, 250ms);  // 250ms seams safe refresh time
  // }
  m_steer_motor->velocity_control(0);  // stops
  hal::print<128>(*console, "Final level: %d\n", m_limit_switch->level());

  m_steer_motor->feedback_request(
    hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
  float stop_angle = m_steer_motor->feedback().angle();

  // hal::print<128>(*console, "Stopped angle: %f\n", stop_angle);
  m_steer_offset = stop_angle - settings.limit_switch_position;
  // hal::print<128>(
  //   *console, "fin position: %f\n",
  //   refresh_actual_state_cache().steer_angle);
}

}  // namespace sjsu::drive
