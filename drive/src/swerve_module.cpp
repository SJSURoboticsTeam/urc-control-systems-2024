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
  hal::v5::strong_ptr<steer_controller> p_steer_controller,
  hal::v5::strong_ptr<propulsion_controller> p_propulsion_controller,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  swerve_module_settings p_settings)
  : settings(p_settings)
  , m_steer_controller(p_steer_controller)
  , m_propulsion_controller(p_propulsion_controller)
  , m_clock(p_clock)
{
  stop();
}

void swerve_module::stop()
{
  m_steer_controller->stop();
  m_propulsion_controller->stop();
}

bool swerve_module::stopped() const
{
  return m_target_state.propulsion_velocity == 0.0f &&
         std::abs(m_actual_state_cache.propulsion_velocity) <=
           settings.velocity_tolerance;
}

void swerve_module::set_target_state(swerve_module_state const& p_target_state)
{
  if (not m_steer_controller->is_homed()) {
    throw hal::resource_unavailable_try_again(this);
  }
  if (!can_reach_state(m_target_state)) {
    auto console = resources::console();
    hal::print(*console, "can't reach state\n");
    throw hal::argument_out_of_domain(this);
  }
  // NAN is to indicate a non-specific angle (angle doesn't matter)
  if (p_target_state.steer_angle == NAN &&
      p_target_state.propulsion_velocity == 0) {
    stop();
  }

  m_target_state = p_target_state;
  m_steer_controller->set_target_position(m_target_state.steer_angle);
  hal::rpm velocity = m_target_state.propulsion_velocity * settings.mps_to_rpm;
  if (settings.drive_forward_clockwise) {
    velocity *= -1;
  }
  m_propulsion_controller->set_target_velocity(velocity);
}

bool swerve_module::can_reach_state(swerve_module_state const& p_state) const
{
  // stoping with out regard for angle
  if ((p_state.propulsion_velocity <= std::abs(settings.max_speed)) &&
      std::isnan(p_state.steer_angle)) {
    return true;
  }
  // steer angle out of range
  if (p_state.steer_angle < settings.min_angle ||
      p_state.steer_angle > settings.max_angle) {
    return false;
  }
  // velocity out of range
  if (std::abs(p_state.propulsion_velocity) > settings.max_speed) {
    return false;
  }
  return true;
}

swerve_module_state swerve_module::get_actual_state_cache() const
{
  return m_actual_state_cache;
}

swerve_module_state swerve_module::refresh_actual_state_cache()
{
  // auto console = resources::console();
  // hal::print(*console, "actual_state:");
  m_actual_state_cache.steer_angle = m_steer_controller->get_actual_postion();
  // hal::print<64>(*console, "%f,", m_actual_state_cache.steer_angle);

  m_actual_state_cache.propulsion_velocity =
    m_propulsion_controller->get_actual_velocity() / settings.mps_to_rpm;
  if (settings.drive_forward_clockwise) {
    m_actual_state_cache.propulsion_velocity *= -1;
  }
  // hal::print<64>(*console, "%f\n", m_actual_state_cache.propulsion_velocity);
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
  m_steer_controller->hard_home();
}

hal::degrees swerve_module::get_steer_offset()
{
  // TODO: remove or implement if needed
  throw hal::operation_not_supported(this);
}

}  // namespace sjsu::drive
