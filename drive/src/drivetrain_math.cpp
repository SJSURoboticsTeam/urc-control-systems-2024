#include <swerve_module.hpp>
#include <vector2d.hpp>
#include <array>
#include <cmath>
#include <cstdlib>
#include <drivetrain_math.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <numbers>
#include <sys/types.h>

namespace sjsu::drive {

using namespace std::chrono_literals;

vector2d chassis_velocities_to_module_vector(
  chassis_velocities const& p_chassis_velocities,
  swerve_module const& p_module)
{
  vector2d transition = p_chassis_velocities.translation;

  float rotational_vel_radians_per_sec =
    p_chassis_velocities.rotational_vel * std::numbers::pi / 180;
  // rotation position vector by 90 degrees is the vector for 1 rad per sec
  vector2d rotation = rotational_vel_radians_per_sec *
                      vector2d::rotate_90_cw(p_module.settings.position);
  return transition + rotation;
}

std::array<vector2d, module_count> chassis_velocities_to_module_vectors(
  chassis_velocities const& p_chassis_velocities,
  std::array<hal::v5::strong_ptr<swerve_module>, module_count> const& p_modules)
{
  std::array<vector2d, module_count> vectors;
  //  convert rotation speed to radians
  float rotational_vel_radians_per_sec =
    p_chassis_velocities.rotational_vel;
  for (unsigned int i = 0; i < vectors.size(); i++) {
    // translation vector is the same
    vector2d transition = p_chassis_velocities.translation;
    // rotation position vector by 90 degrees is the vector for 1 rad per sec
    vector2d rotation = rotational_vel_radians_per_sec *
                        vector2d::rotate_90_cw(p_modules[i]->settings.position);

    vectors[i] = transition + rotation;
  }
  return vectors;
}

static bool cholesky(const float A_[3][3], const float c[3], float x[3])
{

  const float a00 = A_[0][0];
  const float a10 = A_[1][0], a11 = A_[1][1];
  const float a20 = A_[2][0], a21 = A_[2][1], a22 = A_[2][2];

  // A = L * L^T 

  // 1st coloum
  float L00 = sqrtf(a00);
  if (!(L00 > 0.0f)) return false;
  float L10 = a10 / L00;
  float L20 = a20 / L00;

  // 2nd coloum
  float t11 = a11 - L10*L10;
  if (!(t11 > 0.0f)) return false;
  float L11 = sqrtf(t11);
  float L21 = (a21 - L20*L10) / L11;

  // 3rd coloum
  float t22 = a22 - L20*L20 - L21*L21;
  if (!(t22 > 0.0f)) return false;
  float L22 = sqrtf(t22);

  // forward solve
  float y0 = c[0] / L00;
  float y1 = (c[1] - L10*y0) / L11;
  float y2 = (c[2] - L20*y0 - L21*y1) / L22;

  // backward solve
  x[2] = y2 / L22;
  x[1] = (y1 - L21*x[2]) / L11;
  x[0] = (y0 - L10*x[1] - L20*x[2]) / L00;

  return true;
}

chassis_velocities calc_estimated_chassis_velocities(
  std::array<hal::v5::strong_ptr<swerve_module>, module_count> const&
    p_modules){

    size_t n = module_count;

    float x[n];
    float y[n];
    float vix[n];
    float viy[n];

    for (size_t i = 0; i < n; i++){

      const auto& module = *p_modules[i];

      x[i] = module.settings.position.x;
      y[i] = module.settings.position.y;

      const auto state = module.get_actual_state_cache();
      const float speed = state.propulsion_velocity;
      const float angle = state.steer_angle;

      vix[i] = speed * cosf(angle);
      viy[i] = speed * sinf(angle);   
    }

    float a [3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float c [3] = {0,0,0};
    float x_solution [3] = {0,0,0};

    auto accumulate = [&](float r0, float r1, float position, float v){

      //1st coloum
      a[0][0] += r0 * r0; 
      a[1][0] += r0 * r1; 
      a[2][0] += r0 * position;

      //2nd coloum
      a[0][1] += r1 * r0;
      a[1][1] += r1 * r1;
      a[2][1] += r1 * position;

      //3rd coloum
      a[0][2] += position * r0;
      a[1][2] += position * r1;
      a[2][2] += position * position;

      //C-matrix
      c[0] += r0 * v;
      c[1] += r1 * v;
      c[2] += position * v;
    };

    for (size_t i = 0; i < n; i++){
      accumulate(1.0f, 0.0f, -y[i], vix[i]);
      accumulate(0.0f, 1.0f, x[i], viy[i]);
    }

    chassis_velocities results{};

    if (!cholesky(a,c,x_solution)){
      return results;
    }

    results.translation.x = x_solution[0];
    results.translation.y = x_solution[1];
    results.rotational_vel = x_solution[2];

    return results;

    }


swerve_module_state calculate_freest_state(swerve_module const& p_module,
                                           vector2d const& p_target_vector)
{
  float mid_point =
    (p_module.settings.min_angle + p_module.settings.max_angle) / 2.0f;
  swerve_module_state freest_state;
  freest_state.steer_angle =
    modulus_range(vector2d::polar_angle(p_target_vector) * (180 / std::numbers::pi),
                  mid_point - 90,
                  mid_point + 90);
  freest_state.propulsion_velocity = vector2d::length(p_target_vector);
  if (freest_state.steer_angle != modulus_range(vector2d::polar_angle(p_target_vector) * (180 / std::numbers::pi),
                  mid_point - 180,
                  mid_point + 180)) {
    freest_state.propulsion_velocity *= -1;
  }
  return freest_state;
}

swerve_module_state calculate_closest_state(swerve_module const& p_module,
                                            vector2d const& p_target_vector)
{
  // if velocity 0 just keep current angle
  if (vector2d::length(p_target_vector) == 0) {
    return { p_module.get_actual_state_cache().steer_angle, 0 };
  }
  float cur_angle = p_module.get_actual_state_cache().steer_angle;
  swerve_module_state closest_state;
  closest_state.steer_angle =
    modulus_range(vector2d::polar_angle(p_target_vector) * (180 / std::numbers::pi),
                  cur_angle - 90,
                  cur_angle + 90);

  closest_state.propulsion_velocity = vector2d::length(p_target_vector);
  if (modulus_range(vector2d::polar_angle(p_target_vector) * (180 / std::numbers::pi),
                    cur_angle - 180,
                    cur_angle + 180) !=
      closest_state.steer_angle) {
    closest_state.propulsion_velocity *= -1;
  }
  return closest_state;
}

sec calculate_total_interpolation_time(swerve_module const& p_module,
                                       swerve_module_state const& p_end_state)
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
  std::array<hal::v5::strong_ptr<swerve_module>, module_count> const& p_modules,
  std::array<swerve_module_state, module_count> const& p_end_states)
{
  sec max_time = 0.0f;
  for (unsigned int i = 0; i < p_modules.size(); i++) {
    sec time =
      calculate_total_interpolation_time(*(p_modules[i]), p_end_states[i]);
    if (time > max_time) {
      max_time = time;
    }
  }
  return max_time;
}

std::array<swerve_module_state, module_count> scale_down_propulsion_speed(
  std::array<hal::v5::strong_ptr<swerve_module>, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_states)
{
  float portion = 1.0;
  for (unsigned int i = 0; i < p_modules.size(); i++) {
    if (fabsf(p_states[i].propulsion_velocity * portion) >
        p_modules[i]->settings.max_speed) {
      portion = p_modules[i]->settings.max_speed /
                fabsf(p_states[i].propulsion_velocity);
    }
  }
  std::array<swerve_module_state, module_count> scale_down_states;
  for (unsigned int i = 0; i < scale_down_states.size(); i++) {
    scale_down_states[i] = { .steer_angle = p_states[i].steer_angle,
                             .propulsion_velocity =
                               p_states[i].propulsion_velocity * portion };
  }
  return scale_down_states;
}

swerve_module_state interpolate_state(float p_portion,
                                      swerve_module_state const& p_start_state,
                                      swerve_module_state const& p_end_state)
{
  if (p_portion >= 1.0f) {
    return p_end_state;
  } else if (p_portion <= 0.0f) {
    return p_start_state;
  } else {
    return { p_start_state.steer_angle * (1.0f - p_portion) +
               p_end_state.steer_angle * p_portion,
             p_start_state.propulsion_velocity * (1.0f - p_portion) +
               p_end_state.propulsion_velocity * p_portion };
  }
}

std::array<swerve_module_state, module_count> interpolate_states(
  sec p_cycle_time,
  std::array<hal::v5::strong_ptr<swerve_module>, module_count> const& p_modules,
  std::array<swerve_module_state, module_count> const& p_end_states)
{
  std::array<swerve_module_state, module_count> interpolated_states;
  sec interpolation_time =
    calculate_total_interpolation_time(p_modules, p_end_states);
  // prevent div by 0 (already there)
  if (interpolation_time == 0) {
    return p_end_states;
  }

  float portion = p_cycle_time / interpolation_time;
  // can cover full distance
  if (portion >= 1) {
    return p_end_states;
  }
  for (unsigned int i = 0; i < p_modules.size(); i++) {
    interpolated_states[i] = interpolate_state(
      portion, p_modules[i]->get_actual_state_cache(), p_end_states[i]);
  }
  return interpolated_states;
}

float modulus_range(float p_value, float p_lower, float p_upper)
{
  float diff = p_upper - p_lower;
  float offset = p_value - p_lower;
  // if less then push up
  if (offset < 0) {
    int rounds_below = -offset / diff + 1;
    offset += diff * rounds_below;
  }
  // if more
  else if (offset >= diff) {
    int rounds_above = offset / diff;
    offset -= diff * rounds_above;
  }
  return offset + p_lower;
}

hal::time_duration get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}

}  // namespace sjsu::drive