#pragma once
#include "swerve_module.hpp"
#include <array>
namespace sjsu::drive {
class drivetrain
{
public:
  /**
   * @param p_modules the swerve modules of the drivetrain
   */
  drivetrain(std::array<swerve_module&, module_count>& p_modules);
  /**
   * @brief sets the target velocities of the drivetrain
   *
   * @param p_target_state the target velocities
   * @param p_resolve_module_conflicts if false, drivetrain will completely stop
   * if it can't interpolate. If true drivetrain will stop then readjust wheels
   * to get to target state.
   */
  void set_target_state(chassis_velocities p_target_state,
                        bool p_resolve_module_conflicts);
  /**
   * @brief calculates an estimate of the drivetrain velocities based on module readings
   *
   * @return estimate of the drivetrain velocities
   */
  chassis_velocities get_actual_state();
  /**
   * @brief this is the the function to call to update every cycle
   */
  void periodic();
  /**
   * @brief reads sensors and updates accordingly
   */
  void refresh_telemetry();
  /**
   * @brief stops the drivetrain motors
   */
  void stop();
};

}  // namespace sjsu::drive