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
    * @brief Struct for the values that are individual to each servo.
  */
  struct servo_values 
  {
    // for reading value 
    float gear_ratio; 
    // for feedforward 
    float feedforward_clamp; // power needed to keep position at max gravity
    float length; 
    float angle_offset; 
    float weight_beam; 
    float weight_end; 
  };
  /**
    * @brief Set the target position of the servo.
    * @param target_position The target position to set, it is float value in
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
  hal::degrees get_reading_position();

  /**
    * @brief Set the current position of the servo.
    * It will not immediately go to the target position, but will try to reach it using velocity control.
    * @param reading_position The current position to set, it is a hal::degrees value. This is relative to the home position.
  */
  void set_reading_position(hal::degrees reading_position);

  /**
    * @brief Set the target velocity of the servo.
    * The servo will try to reach this velocity using acceleration limits.
    * This may change the clamped speed. 
    * @param target_velocity The target velocity to set, it is a float value.
  */
  void set_target_velocity(float target_velocity);

  /**
   * @brief Set the current velocity of the servo.
   *  This should only be used to set the velocity to 0.
   * @param reading_velocity The current velocity to set, it is a float value.
   */
  void set_reading_velocity(float reading_velocity);

  /**
   * @brief TURNS OFF (Power = 0)
   */
  void stop();


  hal::degrees read_angle(); 

  /**
    * @brief Get the current velocity of the servo.
    * @return The current velocity of the servo as a float value representing degrees per second.
  */
  float get_reading_velocity();

  /**
    * @brief Get the target velocity of the servo.
    * @return The target velocity of the servo as a float value representing degrees per second.
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
    * @brief Update velocity to the target velocity using PID control and feedforward. 
  */
  void update_velocity(); 
  /**
    * @brief Update position to the target position using PID control and feedforward. 
  */
  void update_position(); 
  /**
   * @brief Feedforward values to account for gravity/weight 
   * @return Current feedforward value 
  */
  float position_feedforward();


  /**
    * @brief Set the maximum power the PID controller is allowed to use.
    * @param power The clamped power as a float between 0.0 and 1.0, representing 0% to 100% of maximum possible power.
  */
  void set_pid_clamped_power(float power);

  /**
    * @brief Get the maximum power the PID controller is allowed to use.
    * @return The clamped power as a float between 0.0 and 1.0, representing 0% to 100% of maximum possible power.
  */
  float get_pid_clamped_power();

  /**
    * @brief Sets the power (ignores clamped power) 
    * Use with caution. Check max power beforehand.
    * @param power The power to set the motor to, as a float between -1.0 and 1.0
    * where -1 is the maximum in one direction and 1 is the maximum in the other direction.
  */
  void set_power(float power);

  /**
    * @brief Get the power the servo is using.
    * @return The power as a float between 0.0 and 1.0, representing 0% to 100% of maximum possible power.
  */
  float get_power();

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
    return static_cast<hal::time_duration>(static_cast<long long>(p_time * 1e9f));
  }

  constexpr sec hal_time_duration_to_sec(hal::time_duration p_time)
  {
    return static_cast<float>(p_time.count()) * 1e-9f;
  }

  hal::time_duration get_clock_time(hal::steady_clock& p_clock);


private:
  hal::v5::strong_ptr<sjsu::drivers::h_bridge>
    m_h_bridge;
  hal::v5::strong_ptr<hal::rotation_sensor>
    m_encoder;
  hal::v5::strong_ptr<hal::steady_clock> 
    m_clock;
  hal::u64 m_last_clock_check; 
  status m_reading;
  status m_target;
  PID_settings m_reading_position_settings;
  PID_settings m_reading_velocity_settings;
  PID_prev_values m_PID_prev_velocity_values; 
  PID_prev_values m_PID_prev_position_values; 
  servo_values m_servo_values; 
  float m_clamped_power;
  float m_prev_encoder_value;
  float home_encoder_value;
};

}  // namespace sjsu::perseus