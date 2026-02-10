#pragma once
#include <array>
#include <libhal/pointers.hpp>
#include <swerve_module.hpp>

namespace sjsu::drive {
class drivetrain
{
public:
  /**
   * @param p_modules the swerve modules of the drivetrain
   */
  drivetrain(hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>,
                                            module_count>> p_modules,
             sec p_refresh_rate);
  /**
   * @brief sets the target velocities of the drivetrain
   *
   * @param p_target_state the target velocities
   * @param p_resolve_module_conflicts if false, drivetrain will completely stop
   * if it can't interpolate. If true drivetrain will stop then readjust wheels
   * to get to target state.
   * @return if all the drivetrain can smoothly interpolate
   */
  bool set_target_state(chassis_velocities p_target_state,
                        bool p_resolve_module_conflicts);
  chassis_velocities get_target_state();
  /**
   * @brief calculates an estimate of the drivetrain velocities based on module
   * readings
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
  /**
   * @brief if the drivetrain is at a full stop (or within tolerance of stop)
   * @return if the drivetrain is at a full stop (or within tolerance of stop)
   */
  bool stopped();
  /**
   * @brief if the swerve module angles match the final target state angles (or
   * within tolerance of that)
   * @return if the swerve module angles match the final target state angles (or
   * within tolerance of that)
   */
  bool aligned();
  /**
   * @brief synchronously home, blocks thread until homing is finished.
   */
  void hard_home(hal::v5::strong_ptr<hal::steady_clock> p_clock);
  /**
   * @brief begin homing asynchronously, polling for "homing stop" is done
   * automatically within periodic() so you should not call async_home_begin()
   * without calling periodic() in a loop!
   */
  void async_home_begin();
  /**
   * @brief stops homing immediately on all motors, regardless if they are
   * currently homing or not.
   */
  void async_home_stop();

private:
  hal::v5::strong_ptr<
    std::array<hal::v5::strong_ptr<swerve_module>, module_count>>
    m_modules;
  sec m_refresh_rate;
  std::array<swerve_module_state, module_count> m_final_target_module_states;
  chassis_velocities m_chassis_velocities_estimate;
  chassis_velocities m_target_state;
  bool m_resolve_module_conflicts = false;
  bool m_stopping = false;
  /**
   * @return true if at least one swerve module is currently homing.
   */
  bool async_home_poll();
};

}  // namespace sjsu::drive
