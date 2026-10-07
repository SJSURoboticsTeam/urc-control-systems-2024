#pragma once
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/can.hpp>
#include <libhal/units.hpp>
#include <libhal/error.hpp>

namespace sjsu::drivers {

/**
 * @brief Contains the range settings for the motor
 * 
 */
struct damiao_motor_settings
{
  hal::byte master_id;
  hal::byte can_id;
  float pos_max;
  float pos_min;
  float vel_min;
  float vel_max;
  float Kp_min;
  float Kp_max;
  float Kd_min;
  float Kd_max;
  float torque_min;
  float torque_max;
};

/**
 * @brief This is a data struct that contains the motor's position, velocity and torque
 * 
 */
struct damiao_motor_data {
  float position;
  float velocity;
  float torque;
};

class damiao_motor
{
public:
 /**
  * @brief Construct a new damiao motor object
  * 
  * @param p_can_transceiver Receives a can_transceiver object to send / receive can messages
  * @param p_set Receives damiao_motor_settings struct for motor configuration
  * @param p_clock Receives clock input for deadlines
  * @param p_max_response_time The highest time amount the clock can wait for a response from the motor 
  */
  damiao_motor(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
               struct damiao_motor_settings p_set, hal::v5::strong_ptr<hal::steady_clock> p_clock, hal::time_duration p_max_response_time = std::chrono::milliseconds(500));

  /**
   * @brief Enable the motor 
   * 
   */
  void enable();

  /**
   * @brief Disable the motor
   * 
   */
  void disable();

  /**
   * @brief Use motor's pd controller mode
   * 
   * @param pos Position parameter for the motor (degrees)
   * @param vel Velocity parameter for the motor (RPM)
   * @param Kp Proportional gain for the motor 
   * @param Kd Derivative gain for the motor
   * @param t_ff Torque feed forward for the motor
   */
  void mit(float pos, float vel, float Kp, float Kd, float t_ff);

  /**
   * @brief Use position velocity mode for the motor that goes to the specified position at a constant specified velocity
   * 
   * @param pos Position parameter (degrees)
   * @param vel Velocity parameter (RPM)
   */
  void position_velocity(float pos, float vel);

  /**
   * @brief Use velocity mode for the motor 
   * 
   * @param vel Velocity Parameter (RPM)
   */
  void velocity_start(float vel);

  /**
   * @brief Stop motor if in velocity mode
   * 
   */
  void velocity_stop();

  /**
   * @brief Position hybrid mode for the motor, that takes in a position, velocity and torque current limit
   * 
   * @param pos Position parameter (degrees)
   * @param vel Velocity parameter (RPM)
   * @param torque_current_limit Torque current limit parameter (Range 0 - 1.0)
   */
  void force_position_hybrid(float pos,
                                  float vel,
                                  float torque_current_limit);

  /**
   * @brief Read position, velocity, torque values from the encoder
   * 
   * @return damiao_motor_data 
   */
  damiao_motor_data read_encoder();

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  damiao_motor_settings m_set;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::time_duration m_max_response_time;
  hal::can_message m_recent_mit_frame;
  hal::can_message send_can_data(const hal::can_message& sent_message);
  void mode_set(const uint8_t mode);
};
}  // namespace sjsu::drive