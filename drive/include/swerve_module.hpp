#pragma once

#include "vector2d.hpp"
#include <cmath>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <swerve_structs.hpp>

namespace sjsu::drive {

struct swerve_module_settings
{
  vector2d position = vector2d(NAN,NAN);
  meters_per_sec max_speed = NAN;
  meters_per_sec_per_sec acceleration = NAN;
  deg_per_sec turn_speed = NAN;
  hal::degrees min_angle = NAN;
  hal::degrees max_angle = NAN;
  hal::degrees limit_switch_position = NAN;
  hal::degrees position_tolerance = NAN;
  meters_per_sec velocity_tolerance = NAN;
  sec tolerance_timeout = NAN;
  // If motor turns clockwise inorder to home
  bool home_clockwise = true;
};

class swerve_module
{
public:
  swerve_module_settings settings;

  /**
   * @param p_steer_motor the motor used to control
   * @param p_propulsion_motor the motor
   * @param p_setting module config info
   */
  swerve_module(
    hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_steer_motor,
    hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> p_propulsion_motor,
    hal::v5::strong_ptr<hal::input_pin> p_limit_switch,
    hal::v5::strong_ptr<hal::steady_clock> p_clock,
    swerve_module_settings p_settings);
  /**
   * @brief stops the motors of the module
   */
  void stop();
  /**
   * @brief if the drivetrain is at a full stop (or within tolerance of stop)
   * @return if the drivetrain is at a full stop (or within tolerance of stop)
   */
  bool stopped() const;
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
  bool can_reach_state(swerve_module_state const& p_state) const;

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
  swerve_module_state get_target_state() const;
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
  bool tolerance_timed_out() const;
  /**
   * @brief with run homing in a fixed loop (will not update other motors or get
   * interupted)
   */
  void hard_home();

private:
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> m_steer_motor;
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> m_propulsion_motor;
  hal::v5::strong_ptr<hal::input_pin> m_limit_switch;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  swerve_module_state m_target_state;
  swerve_module_state m_actual_state_cache;
  // the position reading when facing forward using the interface for the steer
  // motor (NAN indicates it has not been homed before)
  hal::degrees m_steer_offset = NAN;

private:
};
}  // namespace sjsu::drive