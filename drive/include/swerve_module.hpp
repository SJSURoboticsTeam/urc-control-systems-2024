#pragma once

#include <cmath>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <limits>
#include <propulsion_controller.hpp>
#include <steer_controller.hpp>
#include <swerve_structs.hpp>
#include <vector2d.hpp>


namespace sjsu::drive {

using namespace std::chrono_literals;

struct swerve_module_settings
{
  vector2d position = vector2d(NAN, NAN);
  meters_per_sec max_speed = 10;
  meters_per_sec_per_sec acceleration = 4.0;
  deg_per_sec turn_speed = 36000.0;
  hal::degrees min_angle = -std::numeric_limits<float>::infinity();
  hal::degrees max_angle = std::numeric_limits<float>::infinity();
  hal::degrees position_tolerance = 5.0;
  meters_per_sec velocity_tolerance = 0.5;
  float mps_to_rpm = 1600;
  sec tolerance_timeout = 0.5;
  // If motor turns clockwise inorder to home
  bool drive_forward_clockwise = true;
};

class swerve_module
{
public:
  swerve_module_settings settings;

  /**
   * @param p_steer_controller steer controller for the module
   * @param p_propulsion_controller propulsion controller for the module
   * @param p_clock steady clock
   * @param p_settings module config info
   */
  swerve_module(
    hal::v5::strong_ptr<steer_controller> p_steer_controller,
    hal::v5::strong_ptr<propulsion_controller> p_propulsion_controller,
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
  hal::v5::strong_ptr<steer_controller> m_steer_controller;
  hal::v5::strong_ptr<propulsion_controller> m_propulsion_controller;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  swerve_module_state m_target_state;
  swerve_module_state m_actual_state_cache;
  hal::time_duration m_tolerance_last_changed = 0ns;
  // true = out of tolerance
  bool m_stable_tolerance_state = false;
};
}  // namespace sjsu::drive
