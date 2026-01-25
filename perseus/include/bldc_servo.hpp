#pragma once
#include <h_bridge.hpp>
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>


using sec = float;

namespace sjsu::perseus {

class bldc_perseus
{

public:
  bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
               hal::v5::strong_ptr<hal::rotation_sensor> p_encoder);

  /**
   * @brief Struct keeps position and velocity status of the servo.
   */
  struct status
  {
    hal::degrees position;
    float power;
    float velocity;
  };

  /**
   * @brief Struct keeps PID settings for the servo.
   */
  struct PID_settings
  {
    float kp = 0.1;
    float ki = 0.1;
    float kd = 0.1;
  };
  /**
   * @brief Struct keeps previous PID settings for the servo.
   * Obtained from update_velocity/position functions
   */
  struct PID_prev_values
  {
    float integral;
    float last_error;
    float prev_dt_time;
  };
  /**
   * @brief Set the target position of the servo.
   * @param target_position The target position to set, it is a float value in
   * degrees.
   */
  void set_target_position(hal::degrees target_position);
  /**
   * @brief Get the target position of the servo.
   * @return Gets the position relative to the home position.
   */
  hal::degrees get_target_position();
  /**
   * @brief Get the current position of the servo.
   * @return Gets the position relative to the home position.
   */
  hal::degrees get_current_position();

  /**
   * @brief Set the target velocity of the servo.
   * The servo will try to reach this velocity using acceleration limits.
   * Target Velocity Units: degrees / second
   * @param target_velocity The target velocity to set, it is a float value.
   */
  void set_target_velocity(float target_velocity);

  /**
   * @brief TURNS OFF (Power = 0)
   */
  void stop();

  /**
   * @brief Get the current velocity of the servo.
   * @return The current velocity of the servo as a float value in ticks per
   * second.
   */
  float get_current_velocity_in_tps();

  /**
   * @brief Get the current velocity of the servo as a percentage of maximum
   * speed.
   * @return The current velocity of the servo as a float value between -1
   * and 1.
   */
  float get_current_velocity_percentage();
  /**
   * @brief Get the target velocity of the servo.
   * @return The target velocity of the servo as a float.
   */
  float get_target_velocity();

  /**
   * @brief Update the PID settings of the servo.
   * @param settings The PID settings to update.
   */
  void update_pid_position(PID_settings settings);

  /**
   * @brief Update the PID settings of the servo.
   * @param settings The PID settings to update.
   */
  void update_pid_velocity(PID_settings settings);

  /**
   * @brief Remembers the current position of the encoder as the home position.
   * This should be called when the servo is homed.
   */
  void home_encoder();

  /**
   * @brief Update velocity to the target velocity using PID control
   */
  void update_velocity();
  /**
   * @brief Update position to the target position using PID control
   */
  void update_position();
  /**
   * @brief get velocity from encoder values
   * prints to terminal
   */
  void get_current_velocity();

  /**
   * @brief Set the maximum power the PID controller is allowed to use.
   * @param power The maximum power as a float between 0.0 and 1.0, representing
   * 0% to 100% of maximum power.
   */
  void set_pid_clamped_power(float power);

  /**
   * @brief Resets the internal time tracking for the servo, this will be done
   * when PID switches between Position and Velocity control.
   */
  void reset_time();

  /**
   * @brief Get the current PID settings of the servo.
   * @return The current PID settings of the servo.
   */
  bldc_perseus::PID_settings get_pid_settings();

  // Helper conversion functions (copied from drivetrain_math.hpp)
  constexpr hal::time_duration sec_to_hal_time_duration(sec p_time)
  {
    return static_cast<hal::time_duration>(
      static_cast<long long>(p_time * 1e9f));
  }

  constexpr sec hal_time_duration_to_sec(hal::time_duration p_time)
  {
    return static_cast<float>(p_time.count()) * 1e-9f;
  }

  hal::time_duration get_clock_time();

  void set_power(float power) {
    m_h_bridge->power(power);
  }

private:
  hal::v5::strong_ptr<sjsu::drivers::h_bridge> m_h_bridge;
  hal::v5::strong_ptr<hal::rotation_sensor> m_encoder;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::u64 m_last_clock_check;
  status m_current;
  status m_target;
  PID_settings m_current_position_settings;
  PID_settings m_current_velocity_settings;
  PID_prev_values m_PID_prev_velocity_values;
  PID_prev_values m_PID_prev_position_values;
  float m_clamped_speed;
  float m_clamped_accel;
  float m_prev_encoder_value;
  float home_encoder_value;
};

}  // namespace sjsu::perseus