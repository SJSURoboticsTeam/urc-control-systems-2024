#include <can_util.hpp>
#include <cstdint>
#include <cstdlib>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <optional>
#include <perseus_bldc.hpp>
#include <resource_list.hpp>
#include <span>

namespace {
enum class action : hal::byte
{
  // top priority
  power_off_reset = 0x0C,  // hard stop the servo to be 0
  heartbeat = 0x0E,        // are you alive

  // actuators
  homing = 0x11,
  set_position_target = 0x12,
  set_position_reading = 0x13,
  set_velocity_target = 0x14,
  set_power = 0x16,
  set_pid_position_config = 0x17,
  set_pid_velocity_config = 0x18,

  // readers
  read_homing_status = 0x21,
  read_position_target = 0x22,
  read_position_reading = 0x23,
  read_velocity_target = 0x24,
  read_velocity_reading = 0x25,
  read_power = 0x26,
  read_pid_position_config = 0x27,
  read_pid_velocity_config = 0x28,

  // servo to servo
  prev_joint_actual_position = 0x41,
  prev_joint_position_response = 0x51,
};
};

namespace sjsu::drivers {

// TODO: implement unimplemented functions

perseus_bldc::perseus_bldc(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::u32 p_can_id,
  hal::time_duration p_max_response_time)
  : m_can_transceiver(p_can_transceiver)
  , m_clock(p_clock)
  , m_can_id(p_can_id)
  , m_reply_message_finder(
      hal::can_message_finder(*p_can_transceiver, p_can_id + 0x100))
  , m_max_response_time(p_max_response_time)
{
  // TODO: run any initialization code (such as killing power)
  kill_power();
}

void perseus_bldc::kill_power()
{
  // TODO: use kill_power command rather than `set_power(0);`
  set_power(0);
}
void perseus_bldc::heart_beat()
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::home()
{
  send({ static_cast<hal::byte>(action::homing) }, 1);
}
bool perseus_bldc::is_homing()
{
  auto reply = send({ static_cast<hal::byte>(action::read_homing_status) }, 1);
  return reply.payload[1];
}
void perseus_bldc::set_target_position(hal::degrees p_target_position)
{
  int8_t exponent = 14;
  auto target_position_int =
    can_util::floating_to_fixed_point_32(p_target_position / 360.0f, exponent);
  auto target_position_array =
    can_util::int32_to_byte_array_big_endian(target_position_int);
  send(
    {
      static_cast<hal::byte>(action::set_position_target),
      static_cast<hal::byte>(exponent),
      target_position_array[0],
      target_position_array[1],
      target_position_array[2],
      target_position_array[3],
    },
    6);
}
bool perseus_bldc::hit_target_position()
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_target_velocity(hal::rpm p_target_velocity
                                       [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_power(float p_portion)
{
  bool sign = p_portion < 0;
  uint16_t duty_cycle = std::abs(p_portion) * 0xFFFF;
  auto duty_cycle_array = can_util::int16_to_byte_array_big_endian(duty_cycle);
  send({ static_cast<hal::byte>(action::set_power),
         sign,
         duty_cycle_array[0],
         duty_cycle_array[1] },
       4);
}
void perseus_bldc::set_position_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::set_velocity_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
hal::degrees perseus_bldc::get_target_position()
{
  throw hal::operation_not_supported(this);
}
hal::degrees perseus_bldc::get_position()
{
  auto message =
    send({ static_cast<hal::byte>(action::read_position_reading) }, 1);
  return can_util::fixed_to_floating_point_32(
           can_util::byte_array_to_int32_big_endian({ message.payload[2],
                                                      message.payload[3],
                                                      message.payload[4],
                                                      message.payload[5] }),
           message.payload[1]) *
         360.0f;
}
hal::rpm perseus_bldc::get_velocity()
{
  throw hal::operation_not_supported(this);
}
float perseus_bldc::read_power()
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::get_position_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}
void perseus_bldc::get_velocity_pid_config(pid_settings const& p_settings
                                           [[maybe_unused]])
{
  throw hal::operation_not_supported(this);
}

hal::can_message perseus_bldc::send(std::array<hal::byte, 8> const& p_payload,
                                    hal::byte const& length)
{
  hal::can_message const payload{
    .id = m_can_id,
    .length = length,
    .payload = p_payload,
  };

  // Send payload
  m_can_transceiver->send(payload);
  // Wait for reply in case of time out
  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (deadline > m_clock->uptime()) {
    auto const message = m_reply_message_finder.find();
    if (message.has_value()) {
      return message.value();
    }
  }
  throw hal::timed_out(this);
}

};  // namespace sjsu::drivers
