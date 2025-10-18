#pragma once

#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>
#include <libhal/units.hpp>
#include <swerve_structs.hpp>


namespace sjsu::drive {

struct swerve_module_settings
{
  vector2d position;
  // the number to add to outputs (and subtract from readings) when using the
  // position interface for the steer motor
  hal::degrees steer_offset;
  meters_per_sec max_speed;
  meters_per_sec_per_sec acceleration;
  deg_per_sec turn_speed;
  hal::degrees min_angle;
  hal::degrees max_angle;
  hal::degrees position_tolerance;
  meters_per_sec velocity_tolerance;
  sec tolerance_timeout;
  bool reversed;
};

class swerve_module
{
public:
  swerve_module_settings settings;
  hal::v5::strong_ptr<hal::input_pin> limit_switch;

  /**
   * @param p_steer_motor the motor used to control
   * @param p_propulsion_motor the motor
   * @param p_setting module config info
   */
  swerve_module(
    hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_steer_motor,
    hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_propulsion_motor,
    hal::v5::strong_ptr<hal::input_pin> p_limit_switch,
    swerve_module_settings p_settings);
  /**
   * @brief stops the motors of the module
   */
  void stop();
  /**
   * @brief if the drivetrain is at a full stop (or within tolerance of stop)
   * @return if the drivetrain is at a full stop (or within tolerance of stop)
   */
  bool stopped();
  /**
   * @brief sets the target module state the module will try to set
   *
   * @param p_target_state the target module state
   */
  void set_target_state(swerve_module_state const& p_target_state);
  /**
   * @brief calculate if a given state is achievable for the module
   *
   * @param p_state the state the module would try to achieve
   * @return if the values are with in tolerances based on settings
   */
  bool can_reach_state(swerve_module_state const& p_state);

  /**
   * @brief gives the cached module state based on most recent readings
   *
   * @return the module state based on most recent readings
   */
  swerve_module_state get_actual_state_cache() const;
  /**
   * @brief reads the encoder values to update the cached module state
   *
   * @return the module state based on readings
   */
  swerve_module_state refresh_actual_state_cache();

  /**
   * @brief gives the current state the module is trying to achieve
   *
   * @return the current state the module is trying to achieve
   */
  swerve_module_state get_target_state();
  /**
   * @brief updates the debounce so the drivetrain stops when a module's states
   * are outside of tolerance for too long
   */
  void update_tolerance_debouncer();
  /**
   * @brief if the the module has been outside of tolerance for too long based
   * on the most recent denouncer reading
   *
   * @return if the the module has been outside of tolerance for too long
   */
  bool tolerance_timed_out();

private:
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> m_steer_motor;
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> m_propulsion_motor;
  swerve_module_state m_target_state;
  swerve_module_state m_actual_state_cache;
private:
};
}  // namespace sjsu::drive