// template from drivers/include/h_bridge.hpp

#pragma once
#include <chrono>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/motor.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

class perseus_bldc
{
public:
  struct PidSettings  // A structure to hold PID (Proportional, Integral,
                      // Derivative)
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
  };
  // addresses to tell it what to do
  // servo_address, byte [0] = actuate/read
  // if byte[0] is actuate -> then bytes[1-7] are giving the number
  enum class actuate : hal::byte
  {
    torque = 0x10,
    max_speed = 0x11,  // driver sends this speed, the PID controller must use
                       // this speed to get to position
    position =
      0x12,  // this driver must get to postion using PID and feedforward
    velocity = 0x13,
  };

  enum class read : hal::byte
  {
    position = 0x20,
    velocity = 0x21 
  };

  // needs to take in can_transceiver and idf so that our CAN network can
  // receive messages from the can_transceiver

  perseus_bldc(hal::v5::strong_ptr<hal::can_transceiver> transceiver,
               hal::v5::strong_ptr<hal::can_identifier_filter>
                 p_filter,  // this filter will allow it to see messages that
                            // only this pegasus cares about.
               hal::v5::strong_ptr<hal::steady_clock> clock,
               int gear_ratio,  // kind of like gear ratio
               hal::u16 servo_address);

  void set_pid(PidSettings const& p_settings);
  hal::u16 get_position();
  /**
   * @brief Derives the target 
  */
  hal::degrees get_velocity();
  // when we say position we mean angle from homing position
  void set_position(hal::u16 degrees);

  void set_velocity(hal::i16 velocity, hal::u8 exp_bits);
  // insread of set position and set_velocity i could just do a get_feedback and
  // parse position and velocity
  // this depends on whether I can get velocity (in rpm as apposed to percent
  // duty cycle)
  // convert torque into (feedforwarded velocity
  // with respect to current angle)

  /**
   * @brief calculate offset velocity from Torque given the current arm
configuration
   * @return updated motor velocity
  */
  hal::u16 calculate_feed_forward(); 
  void set_gear_ratio(float p_ratio);
  void send_message(std::array<hal::byte, 8> const& p_payload);
  void receive_message(std::array<hal::byte, 8>& buffer);

private:
  int m_gear_ratio;
  hal::can_message_finder m_can;
  hal::v5::optional_ptr<hal::steady_clock> m_clock;
  hal::u32 m_device_id;
  std::chrono::milliseconds m_max_response_time;
};
}  // namespace sjsu::drivers
