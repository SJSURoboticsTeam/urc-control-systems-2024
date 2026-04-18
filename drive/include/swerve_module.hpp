#pragma once

#include <vector2d.hpp>
#include <cmath>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <swerve_structs.hpp>

namespace sjsu::drive {

using namespace std::chrono_literals;

struct swerve_module_settings
{
  vector2d position = vector2d(NAN, NAN);
  meters_per_sec max_speed = 10;
  meters_per_sec_per_sec acceleration = 4.0;
  deg_per_sec turn_speed = 36000.0;
  hal::degrees min_angle = -135.0;
  hal::degrees max_angle = 135.0;
  hal::degrees limit_switch_position = NAN;
  hal::degrees position_tolerance = 5.0;
  meters_per_sec velocity_tolerance = 0.5;
  float mps_to_rpm = 60;
  sec tolerance_timeout = 0.5;
  // If motor turns clockwise inorder to home
  bool home_clockwise = true;
  bool drive_forward_clockwise = true;
};

class swerve_module
{
public:
  swerve_module_settings settings;

  /**
   * @param p_steer_servo velocity servo for steer position control and feedback
   * @param p_steer_homing_motor velocity motor for steer homing free spin
   * @param p_prop_motor velocity motor for propulsion velocity control
   * @param p_limit_switch limit switch for homing
   * @param p_clock steady clock
   * @param p_settings module config info
   */
  swerve_module(
    hal::v5::strong_ptr<hal::velocity_servo> p_steer_servo,
    hal::v5::strong_ptr<hal::velocity_motor> p_steer_homing_motor,
    hal::v5::strong_ptr<hal::velocity_motor> p_prop_motor,
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
  bool valid_interpolation(swerve_module_state const& p_state) const;

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
   * @brief run homing in a fixed loop (will not update other motors or get
   * interupted)
   */
  void hard_home();
  /**
   * @brief gets steer encoder offset
   * @return returns encoder reading in degrees when facing forward
   */
  float get_steer_offset();

private:
  hal::degrees get_steer_motor_position();
  void set_steer_motor_position(hal::degrees p_position);
  void set_steer_motor_velocity(hal::rpm p_velocity);

  hal::rpm get_prop_motor_velocity();
  void set_prop_motor_velocity(hal::rpm p_velocity);

  // velocity_servo handles normal steer operation: position set + position read
  hal::v5::strong_ptr<hal::velocity_servo> m_steer_servo;
  // velocity_motor handles steer homing: free spin until limit switch triggers
  // TODO: resource file must provide both m_steer_servo and m_steer_homing_motor
  //       as two interface views of the same underlying steer motor hardware
  hal::v5::strong_ptr<hal::velocity_motor> m_steer_homing_motor;
  hal::v5::strong_ptr<hal::velocity_motor> m_prop_motor;
  hal::v5::strong_ptr<hal::input_pin> m_limit_switch;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  swerve_module_state m_target_state;
  swerve_module_state m_actual_state_cache;
  // the position reading when facing forward using the interface for the steer
  // motor (NAN indicates it has not been homed before)
  hal::degrees m_steer_offset = NAN;
  hal::time_duration m_tolerance_last_changed = 0ns;
  // true = out of tolerance
  bool m_stable_tolerance_state = false;
};
}  // namespace sjsu::drive