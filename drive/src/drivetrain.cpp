#include <array>
#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <swerve_module.hpp>
#include <vector2d.hpp>

namespace sjsu::drive {

drivetrain::drivetrain(
  hal::v5::strong_ptr<
    std::array<hal::v5::strong_ptr<swerve_module>, module_count>> p_modules,
  sec p_refresh_rate)
  : m_modules(p_modules)
{
  m_refresh_rate = p_refresh_rate;
}
bool drivetrain::set_target_state(chassis_velocities p_target_state,
                                  bool p_resolve_module_conflicts)
{
  m_stopping = false;
  m_target_state = p_target_state;
  m_resolve_module_conflicts = p_resolve_module_conflicts;
  // find final target state
  std::array<vector2d, module_count> vectors =
    chassis_velocities_to_module_vectors(m_target_state, *m_modules);
  [[maybe_unused]] auto console = resources::console();
  for (vector2d v : vectors) {
    hal::print<128>(*console, "vec:%f,%f\n", v.x, v.y);
  }

  // calc if final state is in range
  bool can_reach = true;
  for (int i = 0; can_reach && i < module_count; i++) {
    if (vector2d::length_squared(vectors[i]) == 0) {
      // TODO: doesn't seam to work/do anything, fix
      m_final_target_module_states[i] = swerve_module_state(NAN, 0);
    } else {
      m_final_target_module_states[i] =
        calculate_closest_state(*(m_modules->at(i)), vectors[i]);
    }
    // abort if can't reach
    if (!m_modules->at(i)->can_reach_state(m_final_target_module_states[i])) {
      can_reach = false;
    }
  }

  // if can't reach target then set 0 or freest based on if resolve conflicts is
  // on
  if (!can_reach) {
    m_stopping = true;
    if (m_resolve_module_conflicts) {
      for (int i = 0; i < module_count; i++) {
        m_final_target_module_states[i] =
          calculate_freest_state(*(m_modules->at(i)), vectors[i]);
      }
    } else {
      for (int i = 0; i < module_count; i++) {
        m_final_target_module_states[i] =
          m_modules->at(i)->get_actual_state_cache();
        m_final_target_module_states[i].steer_angle =
          std::clamp(m_final_target_module_states[i].steer_angle,
                     m_modules->at(i)->settings.min_angle,
                     m_modules->at(i)->settings.max_angle);
        m_final_target_module_states[i].propulsion_velocity = 0;
      }
    }
  }
  if (m_target_state == chassis_velocities({ 0, 0 }, 0)) {
    m_stopping = true;
  }
  for (swerve_module_state s : m_final_target_module_states) {
    hal::print<128>(
      *console, "final_state:%f,%f\n", s.steer_angle, s.propulsion_velocity);
  }
  return can_reach;
}
chassis_velocities drivetrain::get_target_state()
{
  return m_target_state;
}

chassis_velocities drivetrain::get_state_estimate() const
{
  return m_chassis_velocities_estimate;
}

void drivetrain::periodic()
{
  [[maybe_unused]] auto console = resources::console();
  // hal::print(*console, "Periodic:\n");
  // TODO: deal with out of tolerance modules.

  // refresh telemetry and state
  refresh_telemetry();
  bool drive_stopped = stopped();
  if (drive_stopped) {
    m_stopping = false;
  }

  std::array<swerve_module_state, module_count> next_target_states;
  // if stopping slow down
  if (m_stopping) {
    hal::print(*console, "stopping\n");
    // fill array with 0 vel
    for (int i = 0; i < module_count; i++) {
      // get current angle and no propulsion
      swerve_module& module = *(m_modules->at(i));
      float curent_angle = module.get_actual_state_cache().steer_angle;
      next_target_states[i].steer_angle = curent_angle;
      next_target_states[i].propulsion_velocity = 0;
    }
  }
  // stopped and not aligned with target_final state align
  else if (drive_stopped && !aligned()) {
    hal::print(*console, "stopped & aligning\n");
    // fill array with 0 vel states and target angles
    for (int i = 0; i < module_count; i++) {
      next_target_states[i].steer_angle =
        m_final_target_module_states[i].steer_angle;
      next_target_states[i].propulsion_velocity = 0;
    }
  }
  // else interpolate directly (but check not already at target state)
  else {
    // hal::print(*console, "regular:");
    // return early if target states already final to not clutter the CAN bus
    bool target_matching = true;
    for (int i = 0; i < module_count; i++) {
      swerve_module& module = *(m_modules->at(i));
      swerve_module_state target_state = module.get_target_state();
      if (target_state != m_final_target_module_states[i] ||
          module.tolerance_timed_out()) {
        target_matching = false;
        break;
      }
    }
    if (target_matching) {
      // hal::print(*console, "skip CAN messages already at final state\n");
      return;
    } else {
      hal::print(*console, "interpolating\n");
      next_target_states = m_final_target_module_states;
    }
  }

  // interpolate into modules next target_states
  next_target_states =
    interpolate_states(m_refresh_rate, *m_modules, next_target_states);
  for (int i = 0; i < module_count; i++) {
    swerve_module& module = *(m_modules->at(i));
    module.set_target_state(next_target_states[i]);
  }
}

void drivetrain::refresh_telemetry()
{
  // refresh modules & tolerance debouncers
  for (auto& i : *m_modules) {
    i->refresh_actual_state_cache();
    i->update_tolerance_debouncer();
  }
  // update velocity estimates
  // TODO: uncomment when calc_estimated_chassis_velocities is implemented
  // m_chassis_velocities_estimate =
  // calc_estimated_chassis_velocities(*m_modules);
}

void drivetrain::stop()
{
  for (auto& i : *m_modules) {
    i->stop();
  }
  m_target_state = { { 0, 0 }, 0 };
  m_final_target_module_states = {};
}

bool drivetrain::stopped() const
{
  for (auto& m : *m_modules) {
    if (!m->stopped())
      return false;
  }
  return true;
}
bool drivetrain::aligned() const
{
  for (int i = 0; i < module_count; i++) {
    swerve_module& module = *(m_modules->at(i));
    float target_angle = m_final_target_module_states[i].steer_angle;
    float actual_angle = module.get_actual_state_cache().steer_angle;
    float tolerance = module.settings.position_tolerance;
    if (std::abs(actual_angle - target_angle) > tolerance) {
      return false;
    }
  }
  return true;
}
void drivetrain::hard_home()
{
  for (auto& m : *m_modules) {
    m->hard_home();
  }
}

hal::degrees drivetrain::get_steer_offset(unsigned int p_module_index) const
{
  if (p_module_index >= m_modules->size()) {
    throw hal::argument_out_of_domain(this);
  }
  return m_modules->at(p_module_index)->get_steer_offset();
}
}  // namespace sjsu::drive