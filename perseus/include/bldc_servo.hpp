#pragma once
#include "../../drivers/include/h_bridge.hpp"
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

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
    hal::u16 position;
    hal::i16 velocity;
  };

  /**
    * @brief Struct keeps PID settings for the servo.
    * The PID parameters will be sent over 2 bytes each. Byte 1 would be the decimal part, and Byte 0 the integer part.
  */
  struct PID_settings
  {
    float kp = 0.1;
    float ki = 0.1;
    float kd = 0.1;
  };
  /**
    * @brief Struct keeps previous PID settings for the servo.
    * Obtained from update_velocity/position(_noff) functions 
  */
  struct PID_prev_values 
  {
    float integral; 
    float last_error; 
  }; 
  /**
    * @brief Set the target position of the servo.
    * @param target_position The target position to set, it is a hal::u16 value. This is relative to the home position. 
  */
  void set_target_position(hal::u16 target_position);
  /**
    * @brief Get the target position of the servo.
    * @return Gets the position relative to the home position.
  */
  hal::u16 get_target_position();
  /**
   * @brief Get the current position of the servo.
   * @return Gets the position relative to the home position.
  */
  float get_current_position();

  /**
    * @brief Set the current position of the servo.
    * It will not immediately go to the target position, but will try to reach it using velocity control.
    * @param current_position The current position to set, it is a hal::u16 value. This is relative to the home position.
  */
  void set_current_position(hal::u16 current_position);

  /**
    * @brief Set the target velocity of the servo.
    * The servo will try to reach this velocity using acceleration limits.
    * The velocity is sent from mission control as a hal::u16 value between -100 and 100, representing -100% to 100% of maximum speed.
    * @param target_velocity The target velocity to set, it is a hal::u16 value.
  */
  void set_target_velocity(hal::i16 target_velocity);

  /**
   * @brief Set the current velocity of the servo.
   *  This should only be used to set the velocity to 0.
   * @param current_velocity The current velocity to set, it is a hal::i16 value.
   */
  void set_current_velocity(hal::i16 current_velocity);

  /**
    * @brief Get the current velocity of the servo.
    * @return The current velocity of the servo as a hal::u16 ticks per second.
  */
  hal::u16 get_current_velocity_in_tps();

  /**
    * @brief Get the current velocity of the servo as a percentage of maximum speed.
    * @return The current velocity of the servo as a hal::u16 value between -100 and 100.
  */
  hal::u16 get_current_velocity_percentage();
  /**
    * @brief Get the target velocity of the servo.
    * @return The target velocity of the servo as a hal::u16 value between -100 and 100.
  */
  hal::u16 get_target_velocity();

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
    * @brief Set the maximum speed the servo is allowed to run at.
    * @param target_clamped_speed The maximum speed as a float between 0.0 and 1.0, representing 0% to 100% of maximum speed.
  */
  void set_clamped_speed(hal::u16 target_clamped_speed);

  /**
    * @brief Update velocity 
    * @param settings PID values 
  */
  void update_velocity_noff(PID_settings settings); 
  /**
    * @brief Update position 
    * @param settings PID values 
  */
  void update_position_noff(PID_settings settings); 
   /**
    * @brief get velocity from encoder values 
    * prints to terminal
  */
  void get_current_velocity(); 

public:
  status m_current;
  status m_target;
  PID_settings m_current_position_settings;
  PID_settings m_current_velocity_settings;
  hal::v5::strong_ptr<sjsu::drivers::h_bridge>
    m_h_bridge;  // idk if this can be copied trivially.
  hal::v5::strong_ptr<hal::rotation_sensor>
    m_encoder;            // idk if these are supposed to be pointers or what
  float m_clamped_speed;
  float m_clamped_accel;
  float current_encoder_value;
  PID_prev_values m_PID_prev_velocity_values; 
  PID_prev_values m_PID_prev_position_values; 
  // I think below can be commented out? 
  float total_position_error;
  float total_velocity_error;
  float last_position_error;
  float last_velocity_error;
};

}  // namespace sjsu::perseus