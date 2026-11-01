// cspell:ignore damiao

#pragma once
#include <libhal-util/can.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

/**
 * @brief Contains the range settings for the motor.
 * All of these are configured via damiao's debugger software for the motor
 *
 */
struct damiao_motor_settings
{
  /// @brief This is used as the address of the message frame when the motor
  /// wants to send a message (usually encoder reading feedback) back
  hal::byte master_id = 0x01;

  /// @brief This is the main address used when sending a message frame to the
  /// damiao motor
  hal::u16 can_id = 0x0002;

  /// @brief These are the minimum and maximum values for position (in degrees)
  /// used when the motor uses signed fixed point datatype instead of IEEE
  /// floats (primarily in MIT (pd) mode and reading feedback frame from the
  /// motor)
  /// @brief It is also advised that your position parameter for the motor stay
  /// within these bounds in general, irrespective of the modes so that the
  /// motor's encoder can keep track of it
  hal::degrees minimum_position = 0;
  hal::degrees maximum_position = 360;

  /// @brief These are the minimum and maximum values for velocity (in RPM) used
  /// when the motor uses signed fixed point datatype instead of IEEE floats
  /// (primarily in MIT (pd) mode and reading feedback frame from the motor)
  /// @brief It is also advised that your velocity parameter for the motor stay
  /// within these bounds in general, irrespective of the modes so that the
  /// motor's encoder can keep track of it
  hal::rpm minimum_velocity = 0;
  hal::rpm maximum_velocity = 100;

  /// @brief These are the minimum and maximum values for proportional gain used
  /// when the motor uses signed fixed point datatype instead of IEEE floats
  /// (primarily in MIT (pd) mode)
  float minimum_proportional_gain = 0;
  float maximum_proportional_gain = 500;

  /// @brief These are the minimum and maximum values for derivative gain used
  /// when the motor uses signed fixed point datatype instead of IEEE floats
  /// (primarily in MIT (pd) mode)
  float minimum_derivative_gain = 0;
  float maximum_derivative_gain = 25;

  /// @brief These are the minimum and maximum values for torque (in Nm) used
  /// when the motor uses signed fixed point datatype instead of IEEE floats
  /// (primarily in MIT (pd) mode and reading feedback frame from the motor)
  hal::newton_meter minimum_torque = 0;
  hal::newton_meter maximum_torque = 40;
};

/**
 * @brief This is the parameter struct for mit_pd_mode()
 *
 */
struct mit_pd_mode_parameters
{
  /// @brief Position parameter for the motor (degrees)
  hal::degrees position;
  /// @brief Velocity parameter for the motor (RPM)
  hal::rpm velocity;
  /// @brief Proportional gain for the motor
  float proportional_gain;
  /// @brief Derivative gain for the motor
  float derivative_gain;
  /// @brief Torque feed forward for the motor
  hal::newton_meter torque_feed_forward;
};

/**
 * @brief This is the parameter struct for force_position_hybrid_mode()
 *
 */
struct force_position_hybrid_mode_parameters
{
  /// @brief Position parameter (degrees)
  hal::degrees position;
  /// @brief Velocity parameter (RPM) with float precision of two digits only
  hal::rpm velocity;
  /// @brief Percentage of maximum current allowed by the motor for torque (in
  /// Amperes) (Range 0.000 - 1.000) with float precision of four digits only
  hal::newton_meter torque_current_limit;
};

/**
 * @brief This is a data struct that contains the motor's position, velocity and
 * torque read from the motor
 *
 */
struct damiao_motor_data
{
  /// @brief Position of the motor (in degrees)
  hal::degrees position;
  /// @brief Velocity of the motor (in RPM)
  hal::rpm velocity;
  /// @brief Torque of the motor (in Nm)
  hal::newton_meter torque;
};

/**
 * @brief This the motor driver for the Damiao series motors, tested on
 * DM-J10010L-2EC motor. The motor uses standard CAN 2.0 communication
 *
 */
class damiao_motor
{
public:
  /**
   * @brief Construct a new damiao motor object
   *
   * @param p_can_transceiver Receives a can_transceiver object to send /
   * receive can messages
   * @param p_set Receives damiao_motor_settings struct for motor configuration
   * @param p_clock Receives clock input for deadlines
   * @param p_max_response_time The highest time amount the clock can wait for a
   * response from the motor
   */
  damiao_motor(
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
    damiao_motor_settings& p_settings,
    hal::v5::strong_ptr<hal::steady_clock> p_clock,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(500));

  /**
   * @brief Enable the motor to take in current to the motor's windings.
   *
   * The other APIs take care of enabling the motor before sending commands.
   *
   * Could be used for debugging / testing.
   *
   */
  void enable();

  /**
   * @brief Disable the motor to stop it from doing anything.
   *
   * This will cut power to the motor's windings.
   *
   * If something is being held using the motor's torque it will most likely
   * come loose.
   *
   * Only do this once the motor does not have to supply extra torque and needs
   * to be disabled to save power draw, while also remembering its encoder
   * position to be enabled and used later (The motor does not remember its
   * position once switched off).
   *
   */
  void disable();

  /**
   * @brief Use motor's pd controller mode, which takes the
   * mit_pd_mode_parameters struct declared earlier.
   *
   * This mode can be used if stiffness (proportional gain) and damping
   * (derivative gain) of the motors for robotic joints is crucial and needs to
   * be dynamically tuned as the motor goes to a target position.
   *
   * @param p_param Parameter struct passed by the user for this mode
   */
  void mit_pd_mode(mit_pd_mode_parameters& p_param);

  /**
   * @brief Use position velocity mode for the motor that goes to the specified
   * position at a constant specified velocity.
   *
   * The motor handles the closed loop motion control in this mode, and is best
   * used if the motor simply needs to go to position A to position B precisely
   * at a certain velocity.
   *
   * @param p_position Position parameter (degrees)
   * @param p_velocity Velocity parameter (RPM)
   */
  void position_velocity_mode(hal::degrees p_position, hal::rpm p_velocity);

  /**
   * @brief Use velocity mode for the motor.
   *
   * This is useful for simple continuous motion where the position of the motor
   * is not relevant, and the motor simply needs to spin at a certain velocity.
   *
   * @param p_velocity Velocity Parameter (RPM)
   */
  void velocity_mode(hal::rpm p_velocity);

  /**
   * @brief Position hybrid mode for the motor, that takes in
   * force_position_hybrid_mode_parameters struct declared earlier.
   *
   * This mode is important if the motor needs to watch its current draw (and by
   * relation, the torque).
   *
   * It allows the motor to go to a position at a certain velocity. If the motor
   * hits a resistance requiring more torque(and hence more current draw), this
   * mode will cap the current draw to protect the system.
   *
   * @param p_param Takes in a parameter struct for this mode
   */
  void force_position_hybrid_mode(
    force_position_hybrid_mode_parameters& p_param);

  /**
   * @brief Read the current position, velocity, torque values from the encoder
   *
   * @return Encoder readings for the motor (position, velocity and torque)
   */
  damiao_motor_data read_encoder();

private:
  /// @brief CAN transceiver object passed by the user
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;

  /// @brief message_finder object used for receiving CAN frames from the motor
  hal::can_message_finder m_message_finder;

  /// @brief Settings for the damiao motor passed by the user
  damiao_motor_settings m_settings;

  /// @brief Steady_clock object passed by the user
  hal::v5::strong_ptr<hal::steady_clock> m_clock;

  /// @brief This is the maximum amount of time a CAN transceiver should wait
  /// for a CAN message to be read from the motor
  hal::time_duration m_max_response_time;

  /// @brief This is a standard dummy MIT frame to send in read_encoder() to
  /// read encoder reading, unless changed in mit_pd() mode
  hal::can_message m_recent_mit_frame{
    .length = 8,
    .payload = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 }
  };

  /**
   * @brief Send CAN message to the motor
   * @param p_sent_message Message struct passed
   * @return Return message (in this case usually encoder reading) from the
   * motor
   */
  hal::can_message send_can_data(hal::can_message const& p_sent_message);

  /**
   * @brief Set mode for the motor
   * @param p_mode Mode required for the motor
   */
  void mode_set(uint8_t p_mode);

  /**
   * @brief Convert Convert floating point numbers to 16 or 12 bit fixed point
   * numbers.
   *
   * Useful for mit_pd() mode, which only accepts fixed point.
   *
   * @param p_value The float value to be converted to fixed point
   * @param p_min The minimum value accepted by the motor. (Configured by Damiao
   * software)
   * @param p_max The maximum value accepted by the motor  (Configured by Damiao
   * software)
   * @param p_bit_width The maximum width of the fixed point number accepted by
   * the motor
   * @return The converted fixed point number
   */
  uint16_t float_to_fixed_point(float p_value,
                                float p_minimum,
                                float p_maximum,
                                uint8_t p_bit_width);

  /**
   * @brief Convert fixed point numbers to floating point numbers.
   *
   * Useful as the motor sends encoder readings in fixed point.
   *
   * @param p_value The fixed point number to be converted, given out by the
   * motor.
   * @param p_min The minimum possible value the motor can give out (Configured
   * by Damiao software)
   * @param p_max The maximum possible value the motor can give out (Configured
   * by Damiao software)
   * @param p_bit_width The maximum width of the fixed point number given out by
   * the motor
   * @return The converted floating point number
   */
  float fixed_point_to_float(uint16_t p_value,
                             float p_minimum,
                             float p_maximum,
                             uint8_t p_bit_width);
};
}  // namespace sjsu::drivers
