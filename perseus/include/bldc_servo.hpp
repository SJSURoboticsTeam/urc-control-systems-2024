#pragma once
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

#include <h_bridge.hpp>


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
    float prev_timestamp; 
  }; 
  /**
    * @brief Struct for the values that are individual to each servo.
  */
  struct physical_servo_values 
  {
    // convert from ticks to degrees or mm
    float gear_ratio; 
    // offset of starting angle from 0 (perpendicular to ground)
    float angle_offset; 
    // power needed to keep servo in place when link parallel to ground 
    float fight_gravity; 
    // limits on power for safety
    float high_clamped_value; 
    float low_clamped_value; 
    // whether motor direction is flipped 
    // true: motor spins clockwise when positive power applied 
    // false: motor spins counter-clockwise when positive power applied
    bool clockwise_positive; 
  };
  /**
    * @brief Set the target position of the servo.
    * @param p_target_position The target position to set, it is float value in
    * degrees.
  */
  void set_target_position(hal::degrees p_target_position);
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
    * @param p_reading_position The current position to set, it is a hal::degrees value. This is relative to the home position.
  */
  void set_reading_position(hal::degrees p_reading_position);

  /**
    * @brief Set the target velocity of the servo.
    * The servo will try to reach this velocity using acceleration limits.
    * This may change the clamped speed. 
    * @param p_target_velocity The target velocity to set, it is a float value.
  */
  void set_target_velocity(float p_target_velocity);

  /**
   * @brief Set the current velocity of the servo.
   *  This should only be used to set the velocity to 0.
   * @param p_reading_velocity The current velocity to set, it is a float value.
   */
  void set_reading_velocity(float p_reading_velocity);

  /**
   * @brief TURNS OFF (Power = 0)
   */
  void stop();

  /**
   * @brief Returns the visible angle (ticks adjusted by gear ratio). 
   * @return Angle in hal::degrees 
   */
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
    * @param p_settings The PID settings to update.
  */
  void update_pid_position(PID_settings p_settings);

    /**
    * @brief Update the PID settings of the servo.
    * @param p_settings The PID settings to update.
  */
  void update_pid_velocity(PID_settings p_settings);

  /**
    * @brief Remembers the current position of the encoder as the home position.
    * This should be called when the servo is homed.
  */
  void home_encoder();

  /**
    * @brief Update velocity to the target velocity using PID control and feedforward. 
      @param p_from_scratch A bool indicator of if the target being moved to is new (1) or not (0). 
                          If the target is new, reset integral value to 0. 

  */
  void update_velocity(bool p_from_scratch); 
  /**
    * @brief Update position to the target position using PID control and feedforward. 
      @param p_from_scratch A bool indicator of if the target being moved to is new (1) or not (0). 
                          If the target is new, reset integral value to 0. 
  */
  void update_position(bool p_from_scratch); 
  /**
   * @brief Feedforward values to account for gravity/weight 
   * @return Current feedforward value 
  */
  float position_feedforward();

  /**
   * @brief Set the clamped power in the positive direction. 
   * @param p_power The upper bounds of the power. 
   */
  void set_pos_clamped_power(float p_power);
  /**
   * @brief Returns the clamped power in the positive direction. 
   * @return power (float)
   */
  float get_pos_clamped_power();
  /**
   * @brief Set the clamped power in the negative direction. 
   * @param p_power The lower bounds of the power. 
   */
  void set_neg_clamped_power(float p_power);
  /**
   * @brief Returns the clamped power in the negative direction. 
   * @return power (float)
   */
  float get_neg_clamped_power(); 

  /**
    * @brief Sets the power (ignores clamped power) 
    * Use with caution. Check max power beforehand.
    * @param p_power The power to set the motor to, as a float between -1.0 and 1.0
    * where -1 is the maximum in one direction and 1 is the maximum in the other direction.
  */
  void set_power(float p_power);

  /**
    * @brief Get the power the servo is using.
    * @return The power as a float between 0.0 and 1.0, representing 0% to 100% of maximum possible power.
  */
  float get_power();


  /**
    * @brief Set the servo's current action 
    * @param p_action can_perseus::action value to be set 
  */
  void set_active_action(uint32_t p_action); 
  
  /**
    * @brief Get the servo's current action 
    * @return Returns the current action as a can_perseus::action 
  */
  uint32_t get_active_action(); 

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

  void periodic_action(bool p_new_action);

  /**
   * @brief Set the angle offset of the servo.
   * @param p_angle_offset The new angle offset (float).
   */
  void set_angle_offset(float p_angle_offset);
  /**
   * @brief Get the angle offset of the servo.
   * @return The current angle offset (float).
   */
  float get_angle_offset();

  /**
   * @brief Set the previous joint's recorded position. 
   * @param p_prev_joint_pos The previous joint's position (float).
   */
  void set_prev_joint_position(float p_prev_joint_pos); 
  /**
   * @brief Get the previous joint's recorded position.
   * @return The previous joint's position (float).
   */
  float get_prev_joint_position(); 

  /**
   * @brief Get actual position of the servo (0 = perpendicular to ground). 
  */
  float get_actual_position(); 

  /**
   * @brief Set servo values
   * @param p_physical_servo_values The values to set (physical_servo_values class) 
  */
  void set_physical_servo_values(physical_servo_values p_physical_servo_values); 

  /**
   * @brief Get servo values object
   * 
   * @return physical_servo_values 
   */
  physical_servo_values get_physical_servo_values(); 

  hal::time_duration get_clock_time(hal::steady_clock& p_clock);


private:
  hal::v5::strong_ptr<sjsu::drivers::h_bridge>
    m_h_bridge;
  hal::v5::strong_ptr<hal::rotation_sensor>
    m_encoder;
  hal::v5::strong_ptr<hal::steady_clock> 
    m_clock;
  hal::u64 m_last_clock_check; 
  status m_target;
  PID_settings m_reading_position_settings;
  PID_settings m_reading_velocity_settings;
  PID_prev_values m_PID_prev_velocity_values; 
  PID_prev_values m_PID_prev_position_values; 
  physical_servo_values m_physical_servo_values; 
  float m_actual_position; 
  float m_prev_joint_position; 
  float m_active_power; 
  uint32_t m_active_action; 
};

}  // namespace sjsu::perseus