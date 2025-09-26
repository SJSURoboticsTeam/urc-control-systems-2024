#include "../include/drivetrain_math.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <libhal/units.hpp>
#include <numbers>
#include <sys/types.h>
namespace sjsu::drive {

using namespace std::chrono_literals;

std::array<vector2d, module_count> chassis_velocities_to_module_vectors(
  chassis_velocities p_chassis_velocities,
  std::array<swerve_module, module_count>& p_modules)
{
  std::array<vector2d, module_count> vectors;
  //  convert rotation speed to radians
  float rotational_vel_radians_per_sec =
    p_chassis_velocities.rotational_vel * std::numbers::pi / 180;
  for (uint i = 0; i < vectors.size(); i++) {
    // translation vector is the same
    vector2d transition = p_chassis_velocities.translation;
    // rotation position vector by 90 degrees is the vector for 1 rad per sec
    vector2d rotation = rotational_vel_radians_per_sec *
                        vector2d::rotate_90_cw(p_modules[i].settings.position);

    vectors[i] = transition + rotation;
  }
  return vectors;
}

float module_validity_strain_score(
  std::array<swerve_module, module_count>& p_modules,
  std::array<vector2d, module_count> p_vectors)
{
  // current calculation only works if all modules are the same distance to the
  // center
  vector2d transition(0, 0);
  // calc overall translation and remove from vectors
  for (auto& v : p_vectors) {
    transition = transition - (v / p_vectors.size());
  }
  for (auto& v : p_vectors) {
    v = v - transition;
  }
  // calc overall turn and remove from vectors
  float turn_speed = 0.0;  // in radians
  for (int i = 0; i < module_count; i++) {
    // get turn vector for 1 rad persec (reference)
    vector2d ref_vector =
      vector2d::rotate_90_cw(p_modules[i].settings.position);
    // get scale of turn vector projected onto refrence vector
    turn_speed -=
      vector2d::dot(ref_vector, p_vectors[i]) / vector2d::length(ref_vector);
  }
  turn_speed /= module_count;  // average it out
  // remove from vectors
  for (int i = 0; i < module_count; i++) {
    // get turn vector for 1 rad persec (reference)
    vector2d ref_vector =
      vector2d::rotate_90_cw(p_modules[i].settings.position);
    // calc final vector
    p_vectors[i] = p_vectors[i] - (ref_vector * turn_speed);
  }
  // the arbitrary function for strain
  // TODO: get a better function for strain from mechanical
  float strain = 0.0;
  for (auto v : p_vectors) {
    strain += vector2d::length_squared(v);
  }
  return strain;
}

chassis_velocities calc_estimated_chassis_velocities(
  std::array<swerve_module, module_count>& p_modules);

swerve_module_state calculate_freest_state(swerve_module& p_module,
                                           vector2d p_target_vector);

swerve_module_state calculate_closest_state(swerve_module& p_module,
                                            vector2d p_target_vector);

sec calculate_total_interpolation_time(swerve_module& p_module,
                                       swerve_module_state p_end_state)
{
  // using a relatively primitive calculation for now

  // calc speed interpolation time
  meters_per_sec speed_diff =
    fabsf(p_module.get_actual_state_cache().propulsion_velocity -
          p_end_state.propulsion_velocity);
  sec propulsion_transition_time = speed_diff / p_module.settings.acceleration;

  // calc angle interpolation time
  hal::degrees angle_diff = fabsf(
    p_module.get_actual_state_cache().steer_angle - p_end_state.steer_angle);
  sec steer_transition_time = angle_diff / p_module.settings.turn_speed;

  if (propulsion_transition_time > steer_transition_time) {
    return propulsion_transition_time;
  } else {
    return steer_transition_time;
  }
}

sec calculate_total_interpolation_time(
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_end_states)
{
  sec max_time = 0.0f;
  for (uint i = 0; i < p_modules.size(); i++) {
    sec time =
      calculate_total_interpolation_time(p_modules[i], p_end_states[i]);
    if (time > max_time) {
      max_time = time;
    }
  }
  return max_time;
}

std::array<swerve_module_state, module_count> scale_down_propulsion_speed(
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_states)
{
  float portion = 1.0;
  for (uint i = 0; i < p_modules.size(); i++) {
    if (fabsf(p_states[i].propulsion_velocity * portion) >
        p_modules[i].settings.max_speed) {
      portion = p_modules[i].settings.max_speed /
                fabsf(p_states[i].propulsion_velocity);
    }
  }
  std::array<swerve_module_state, module_count> scale_down_states;
  for (uint i = 0; i < scale_down_states.size(); i++) {
    scale_down_states[i] = { .steer_angle = p_states[i].steer_angle,
                             .propulsion_velocity =
                               p_states[i].propulsion_velocity * portion };
  }
  return scale_down_states;
}

swerve_module_state interpolate_state(float p_portion,
                                      swerve_module_state p_start_state,
                                      swerve_module_state p_end_state);

std::array<swerve_module_state, module_count> interpolate_states(
  sec p_cycle_time,
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_end_states)
{
  std::array<swerve_module_state, module_count> interpolated_states;
  float portion =
    calculate_total_interpolation_time(p_modules, p_end_states) / p_cycle_time;
  for (uint i = 0; i < p_modules.size(); i++) {
    interpolated_states[i] = interpolate_state(
      portion, p_modules[i].get_actual_state_cache(), p_end_states[i]);
  }
  return interpolated_states;
}
}  // namespace sjsu::drive