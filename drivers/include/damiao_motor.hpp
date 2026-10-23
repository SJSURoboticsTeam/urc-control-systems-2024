 #pragma once
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/can.hpp>
#include <libhal/units.hpp>
#include <libhal/error.hpp>

namespace sjsu::drivers {

/**
 * @brief Contains the range settings for the motor. 
 * All of these are configured via damiao's debugger software for the motor
 * 
 */
struct damiao_motor_settings
{
  /// @brief This is used as the address of the message frame when the motor wants to send a message (usually encoder reading feedback) back
  hal::byte master_id;           

  /// @brief This is the main address used when sending a message frame to the damiao motor
  hal::byte can_id;               

  /// @brief These are the minimum and maximum values for position (in degrees) used when the motor uses signed fixed point datatype instead of IEEE floats (primarily in MIT (pd) mode and reading feedback frame from the motor)
  /// @brief It is also advised that your position parameter for the motor stay within these bounds in general, irrespective of the modes so that the motor's encoder can keep track of it
  float minimum_position;         
  float maximum_position;

  /// @brief These are the minimum and maximum values for velocity (in RPM) used when the motor uses signed fixed point datatype instead of IEEE floats (primarily in MIT (pd) mode and reading feedback frame from the motor)
  /// @brief It is also advised that your velocity parameter for the motor stay within these bounds in general, irrespective of the modes so that the motor's encoder can keep track of it
  float minimum_velocity;
  float maximum_velocity;

  /// @brief These are the minimum and maximum values for proportional gain used when the motor uses signed fixed point datatype instead of IEEE floats (primarily in MIT (pd) mode)
  float minimum_proportional_gain;
  float maximum_proportional_gain;

  /// @brief These are the minimum and maximum values for derivative gain used when the motor uses signed fixed point datatype instead of IEEE floats (primarily in MIT (pd) mode)
  float minimum_derivative_gain;
  float maximum_derivative_gain;

  /// @brief These are the minimum and maximum values for torque (in Nm) used when the motor uses signed fixed point datatype instead of IEEE floats (primarily in MIT (pd) mode and reading feedback frame from the motor)
  float minimum_torque;
  float maximum_torque;
};

/**
 * @brief This is the parameter struct for mit_pd_mode()
 * 
 */
struct mit_pd_mode_parameters {
  /// @brief Position parameter for the motor (degrees)
  float position;
  /// @brief Velocity parameter for the motor (RPM)
  float velocity;
  /// @brief Proportional gain for the motor
  float proportional_gain;
  /// @brief Derivative gain for the motor
  float derivative_gain;
  /// @brief Torque feed forward for the motor
  float torque_feed_forward;
};

/**
 * @brief This is the parameter struct for force_position_hybrid_mode()
 * 
 */
struct force_position_hybrid_mode_parameters {
  /// @brief Position parameter (degrees)
  float position;
  /// @brief Velocity parameter (RPM) with float precision of two digits only
  float velocity;
  /// @brief Percentage of maximum current allowed by the motor for torque (in Amperes) (Range 0.000 - 1.000) with float precision of four digits only
  float torque_current_limit;
};

/**
 * @brief This is a data struct that contains the motor's position, velocity and torque read from the motor
 * 
 */
struct damiao_motor_data {
  /// @brief Position of the motor (in degrees)
  float position;
  /// @brief Velocity of the motor (in RPM)
  float velocity;
  /// @brief Torque of the motor (in Nm)
  float torque;
};

/**
 * @brief Damiao motor class
 * 
 */
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
   * @brief Enable the motor to turn / take commands. This needs to be done once at the start, before any of the other APIs are called. 
   * 
   */
  void enable();

  /**
   * @brief Disable the motor to stop it from doing anything.
   * 
   */
  void disable();

  /**
   * @brief Use motor's pd controller mode
   * 
   * @param p_param Parameter struct passed by the user for this mode
   */
  void mit_pd_mode(mit_pd_mode_parameters p_param);

  /**
   * @brief Use position velocity mode for the motor that goes to the specified position at a constant specified velocity
   * 
   * @param p_position Position parameter (degrees)
   * @param p_velocity Velocity parameter (RPM)
   */
  void position_velocity_mode(float p_position, float p_velocity);

  /**
   * @brief Use velocity mode for the motor 
   * 
   * @param p_velocity Velocity Parameter (RPM)
   */
  void velocity_mode_start(float p_velocity);

  /**
   * @brief Stop motor if in velocity mode
   * 
   */
  void velocity_mode_stop();

  /**
   * @brief Position hybrid mode for the motor
   * 
   * @param p_param Takes in a parameter struct for this mode
   */
  void force_position_hybrid_mode(force_position_hybrid_mode_parameters p_param);

  /**
   * @brief Read position, velocity, torque values from the encoder
   * 
   * @return Encoder readings for the motor (position, velocity and torque)
   */
  damiao_motor_data read_encoder();

private:
  /// @brief CAN transciever object passed by the user
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;

  /// @brief Settings for the damiao motor passed by the user
  damiao_motor_settings m_set;

  /// @brief Steady_clock object passed by the user
  hal::v5::strong_ptr<hal::steady_clock> m_clock;

  /// @brief This is the maximum amount of time a CAN transceiver should wait for a CAN message to be read from the motor
  hal::time_duration m_max_response_time;
  
  /// @brief This is a standard dummy MIT frame to send in read_encoder() to read encoder reading, unless changed in mit_pd() mode
  hal::can_message m_recent_mit_frame;

  /// @brief Send CAN message to the motor
  /// @param p_sent_message Message struct passed 
  /// @return Return message (in this case usually encoder reading) from the motor
  hal::can_message send_can_data(const hal::can_message& p_sent_message);

  /// @brief Set mode for the motor 
  /// @param p_mode Mode required for the motor
  void mode_set(const uint8_t p_mode);
};
}  // namespace sjsu::drive