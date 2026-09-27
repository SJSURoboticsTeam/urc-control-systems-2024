// template from drivers/include/h_bridge.hpp

#pragma once
#include <array>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/motor.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

using namespace std::chrono_literals;

struct pid_settings  // A structure to hold PID (Proportional, Integral,
                     // Derivative)
{
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

class perseus_bldc
{
public:
  perseus_bldc(hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
               hal::v5::strong_ptr<hal::steady_clock> p_clock,
               hal::u32 p_can_id,
               hal::time_duration p_max_response_time = 500ms);
  // currently inorder of messages on CAN ID doc
  /**
   * @brief kills power, sets voltage to 0
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void kill_power();
  /**
   * @brief used to check that perseus motor communication is responsive
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void heart_beat();
  /**
   * @brief run homing sequence to adjust position
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void home();
  /**
   * @brief check if currently running a homing sequence
   * @returns if currently running a homing sequence
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  bool is_homing();
  /**
   * @brief set target position
   * @param p_target_position new target of positional pid
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_target_position(hal::degrees p_target_position);
  /**
   * @brief check if target position has been hit since target position was
   * given
   * @returns if target position has been hit since target position was given
   * (also true if no target position is active)
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  bool hit_target_position();
  /**
   * @brief sets encoder position to make new frame of reference
   * @param p_position postion is in new frame of reference
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_encoder_position(hal::degrees p_position);
  /**
   * @brief set target velocity
   * @param p_target_velocity new target of velocity pid
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_target_velocity(hal::rpm p_target_velocity);
  /**
   * @brief set voltage as portion of max
   * @param p_portion motor voltage as portion of max voltage, must be a value
   * from -1 to 1 (inclusive)
   * @throws hal::argument_out_of_domain - p_portion was not from -1 to 1
   * (inclusive)
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_power(float p_portion);
  /**
   * @brief set the constants for positional PID
   * @param p_settings new positional PID constants
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_position_pid_config(pid_settings const& p_settings);
  /**
   * @brief set the constants for velocity PID
   * @param p_settings new velocity PID constants
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void set_velocity_pid_config(pid_settings const& p_settings);
  /**
   * @brief get target position
   * @returns target position of positional PID
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  hal::degrees get_target_position();
  /**
   * @brief get position
   * @returns encoder position in established frame of reference
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  hal::degrees get_position();
  // hal::rpm get_target_velocity();
  /**
   * @brief get velocity based on encoder reading
   * @returns velocity based on encoder reading
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  hal::rpm get_velocity();
  /**
   * @brief get voltage portion of max being supplied to motor
   * @returns voltage portion of max being supplied to motor
   */
  float read_power();
  /**
   * @brief get the constants for positional PID
   * @param p_settings positional PID constants
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void get_position_pid_config(pid_settings const& p_settings);
  /**
   * @brief get the constants for velocity PID
   * @param p_settings velocity PID constants
   * @throws hal::timed_out - perseus motor controller doesn't respond
   */
  void get_velocity_pid_config(pid_settings const& p_settings);

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::u32 m_can_id;
  hal::can_message_finder m_reply_message_finder;
  hal::time_duration m_max_response_time;

  hal::can_message send(std::array<hal::byte, 8> const& p_payload,
                        hal::byte const& length);
};
}  // namespace sjsu::drivers
