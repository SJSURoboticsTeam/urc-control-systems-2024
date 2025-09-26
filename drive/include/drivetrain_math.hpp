#include "swerve_module.hpp"
#include "vector2d.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <libhal/units.hpp>
#include <numbers>
#include <sys/types.h>
namespace sjsu::drive {

using namespace std::chrono_literals;

// chassis speed to states
/**
 * @brief converts chassis velocities into ideal velocity vectors for the given
 * modules
 *
 * @param p_chassis_velocities the target chassis velocities
 * @param p_modules modules velocities are being calculated for
 *
 * @return ideal velocity vectors in meters per second in the, they are returned
 * in the order of their corelating modules
 */
std::array<vector2d, module_count> chassis_velocities_to_module_vectors(
  chassis_velocities p_chassis_velocities,
  std::array<swerve_module, module_count>& p_modules);


//will likely be unused for this rover
/**
 * @brief calculates a score that represents how much conflict there is between
 * the modules
 */
float module_validity_strain_score(
  std::array<swerve_module, module_count>& p_modules,
  std::array<vector2d, module_count> p_vectors);

/**
 * @brief calculates an estimate of the chassis velocities based on the cached
 * module states
 *
 * @param p_modules modules used to form the estimate
 * @return chassis velocities estimate
 */
chassis_velocities calc_estimated_chassis_velocities(
  std::array<swerve_module, module_count>& p_modules);

/**
 * @brief calculates the swerve state with the most angular freedom either side
 * that generates the target velocity vector. It doesn't calculate it can reach
 * that position or not.
 *
 * @param p_module module the state is beging generated for
 * @param p_target_vector the velocity vector in meters per second
 * @return state with most angular freedom
 */
swerve_module_state calculate_freest_state(swerve_module& p_module,
                                           vector2d p_target_vector);
/**
 * @brief calculates the swerve state that requires the least steer movement to
 * get tothat generates the target velocity vector. It doesn't calculate it can
 * reach that position or not.
 *
 * @param p_module module the state is beging generated for
 * @param p_target_vector the velocity vector in meters per second
 * @return state that requires the least steer movement to get to
 */
swerve_module_state calculate_closest_state(swerve_module& p_module,
                                            vector2d p_target_vector);

/**
 * @brief calculates estimation of time to interpolate the module to a state
 *
 * @param p_module module being interpolated
 * @param p_end_state state the modules should be in by the end
 * @return estimation of time to interpolate the modules in seconds
 */
sec calculate_total_interpolation_time(swerve_module& p_module,
                                       swerve_module_state p_end_state);

/**
 * @brief calculates estimation of time to interpolate all modules to a
 * corelating list of states
 *
 * @param p_modules modules being interpolated
 * @param p_end_states states the modules should be in by the end
 * @return estimation of time to interpolate all modules in seconds
 */
sec calculate_total_interpolation_time(
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_end_states);

/**
 * @brief scales swerve states down keep the a modules state velocity within
 * it's max velocity
 *
 * @param p_modules the modules states are being scaled for
 * @param p_states the states to be scaled down
 * @return the states scaled down if needed
 */
std::array<swerve_module_state, module_count> scale_down_propulsion_speed(
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_states);

/**
 * @brief calculates the module state mid interpolation
 *
 * @param p_portion how much the interpolation progressed as a portion from 0 to
 * 1
 * @param p_start_state the state at the start of the interpolation
 * @param p_end_state the state at the end of the interpolation
 * @return the module state at the the portion in the interpolation
 */
swerve_module_state interpolate_state(float p_portion,
                                      swerve_module_state p_start_state,
                                      swerve_module_state p_end_state);

/**
 * @brief calculates next module states to target while interpolating
 *
 * @param p_cycle_time time between time modules would be updated
 * @param p_modules the state at the start of the interpolation
 * @param p_end_state the state at the end of the interpolation
 */
std::array<swerve_module_state, module_count> interpolate_states(
  sec p_cycle_time,
  std::array<swerve_module, module_count>& p_modules,
  std::array<swerve_module_state, module_count> p_end_states);
}  // namespace sjsu::drive