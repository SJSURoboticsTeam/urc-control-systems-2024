#include "resource_list.hpp"
#include "swerve_module.hpp"
#include <array>
#include <bit>
#include <cstdint>
#include <libhal-util/bit.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <span>
#include <sys/_types.h>
#include <sys/types.h>

namespace {
enum class can_message_id : uint32_t
{
  set_chassis_velocities = 0x0C,
  set_chassis_velocities_reply = 0x0D,
  heartbeat = 0x0E,
  heartbeat_reply = 0x0F,
  homing_sequence = 0x110,
  homing_sequence_reply = 0x111,
  get_offset = 0x112,
  get_offset_reply = 0x113,
  get_estimated_velocities = 0x114,
  get_estimated_velocities_reply = 0x115,
  config = 0x119,
  config_ack = 0x11A,
};
}

namespace sjsu::drive {

mission_control_manager::mission_control_manager(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver)
  : m_can_transceiver(p_can_transceiver)
  , m_heartbeat_message_finder(
      hal::can_message_finder(*m_can_transceiver,
                              static_cast<uint32_t>(can_message_id::heartbeat)))
  , m_set_chassis_velocities_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::set_chassis_velocities)))
  , m_get_chassis_velocities_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::get_estimated_velocities)))
  , m_homing_request_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::homing_sequence)))
  , m_get_offset_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::get_offset)))
{
}

float mission_control_manager::fixed_point_16_to_float(
  int16_t p_fixed_point_num,
  int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  float floating_num = p_fixed_point_num;
  if (p_expo < 0) {
    return floating_num / shifted_expo;
  } else {
    return floating_num * shifted_expo;
  }
}
int16_t mission_control_manager::float_to_fixed_point_16(
  float p_floating_point_num,
  int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  if (p_expo > 0) {
    p_floating_point_num /= shifted_expo;
  } else {
    p_floating_point_num *= shifted_expo;
  }
  // TODO: consider implementing rounding
  return static_cast<int16_t>(p_floating_point_num);
}

int16_t mission_control_manager::byte_array_to_int16(
  std::array<hal::byte, 2> p_array)
{
  return (static_cast<int16_t>(p_array[0]) << 8) |
         static_cast<int16_t>(p_array[1]);
}
std::array<hal::byte, 2> mission_control_manager::int16_to_byte_array(
  int16_t p_num)
{
  uint16_t unum = std::bit_cast<uint16_t>(p_num);
  std::array<hal::byte, 2> byte_array = { static_cast<uint8_t>(unum >> 8),
                                          static_cast<uint8_t>(unum) };
  return byte_array;
}

float mission_control_manager::fixed_point_32_to_float(
  int32_t p_fixed_point_num,
  int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  float floating_num = p_fixed_point_num;
  if (p_expo < 0) {
    return floating_num / shifted_expo;
  } else {
    return floating_num * shifted_expo;
  }
}
int32_t mission_control_manager::float_to_fixed_point_32(
  float p_floating_point_num,
  int p_expo)
{
  unsigned int shifted_expo = 1 << abs(p_expo);
  if (p_expo > 0) {
    p_floating_point_num /= shifted_expo;
  } else {
    p_floating_point_num *= shifted_expo;
  }
  // TODO: consider implementing rounding
  return static_cast<int32_t>(p_floating_point_num);
}

int32_t mission_control_manager::byte_array_to_int32(
  std::array<hal::byte, 4> p_array)
{
  return (static_cast<int32_t>(p_array[0]) << 24) |
         (static_cast<int32_t>(p_array[1]) << 16) |
         (static_cast<int32_t>(p_array[2]) << 8) |
         (static_cast<int32_t>(p_array[3]) << 0);
}
std::array<hal::byte, 4> mission_control_manager::int32_to_byte_array(
  int32_t p_num)
{
  uint32_t unum = std::bit_cast<uint32_t>(p_num);
  std::array<hal::byte, 4> byte_array = { static_cast<uint8_t>(unum >> 24),
                                          static_cast<uint8_t>(unum >> 16),
                                          static_cast<uint8_t>(unum >> 8),
                                          static_cast<uint8_t>(unum) };
  return byte_array;
}

// returns most recent velocity request
std::optional<chassis_velocities_request>
mission_control_manager::read_set_velocity_request()
{
  auto console = resources::console();
  std::optional<hal::can_message> velocity_request_message;
  while (true) {
    std::optional<hal::can_message> check_velocity_request_message =
      m_set_chassis_velocities_message_finder.find();
    if (check_velocity_request_message) {
      // hal::print(
      //   *console,
      //   "\nmessage passed===============================================");
      if (check_velocity_request_message->length == 7) {
        velocity_request_message = check_velocity_request_message;
      }
    } else {
      break;
    }
  }
  if (not velocity_request_message) {
    return std::nullopt;
  }
  chassis_velocities velocities;
  auto& payload = velocity_request_message->payload;
  velocities.translation.x = fixed_point_16_to_float(
    byte_array_to_int16({ payload[0], payload[1] }), -12);
  velocities.translation.y = fixed_point_16_to_float(
    byte_array_to_int16({ payload[2], payload[3] }), -12);
  velocities.rotational_vel = fixed_point_16_to_float(
    byte_array_to_int16({ payload[4], payload[5] }), -6);
  chassis_velocities_request cvr = { .chassis_vels = velocities,
                                     .module_conflicts =
                                       (bool)(payload[6] & 0x01) };
  hal::print<64>(*console,
                 "\n%x,%x,%x",
                 byte_array_to_int16({ payload[0], payload[1] }),
                 byte_array_to_int16({ payload[2], payload[3] }),
                 byte_array_to_int16({ payload[4], payload[5] }));
  return cvr;
}

void mission_control_manager::reply_set_velocity_request(
  chassis_velocities_request const& p_chassis_vel_req)
{
  auto x_vel_array = int16_to_byte_array(
    float_to_fixed_point_16(p_chassis_vel_req.chassis_vels.translation.x, -12));
  auto y_vel_array = int16_to_byte_array(
    float_to_fixed_point_16(p_chassis_vel_req.chassis_vels.translation.y, -12));
  auto rot_vel_array = int16_to_byte_array(
    float_to_fixed_point_16(p_chassis_vel_req.chassis_vels.rotational_vel, -6));
  hal::can_message reply{ .id = static_cast<uint32_t>(
                            can_message_id::set_chassis_velocities_reply),
                          .length = 7,
                          .payload = { x_vel_array[0],
                                       x_vel_array[1],
                                       y_vel_array[0],
                                       y_vel_array[1],
                                       rot_vel_array[0],
                                       rot_vel_array[1],
                                       p_chassis_vel_req.module_conflicts } };
  m_can_transceiver->send(reply);
}

// return true if homing requested;
bool mission_control_manager::read_homing_request()
{
  bool requested = false;
  while (true) {
    std::optional<hal::can_message> message =
      m_homing_request_message_finder.find();
    if (message) {
      auto console = resources::console();
      // hal::print(*console,"\nmessage
      // passed===============================================");
      if (message->length == 0) {
        requested = true;
      }
    } else {
      return requested;
    }
  }
}

void mission_control_manager::reply_homing_request()
{
  m_can_transceiver->send(
    { .id = static_cast<uint32_t>(can_message_id::homing_sequence_reply),
      .length = 0 });
}

void mission_control_manager::fulfill_data_requests(
  drivetrain const& p_drivetrain)
{
  // handle offset requests
  hal::byte offsets_requested = 0;
  std::optional<hal::can_message> offset_request_message;
  do {
    offset_request_message = m_get_offset_message_finder.find();
    if (offset_request_message && offset_request_message->length == 1) {
      offsets_requested |= 1 << offset_request_message->payload[0];
    }
  } while (offset_request_message);
  hal::byte i = 0;
  while (offsets_requested >> i) {
    if ((offsets_requested >> i) & 1) {
      // send message
      std::array<hal::byte, 4> offset_bytes = int32_to_byte_array(
        float_to_fixed_point_32(p_drivetrain.get_steer_offset(i), -22));
      hal::can_message
        offset_request_reply = { .id = static_cast<uint32_t>(
                                   can_message_id::heartbeat_reply),
                                 .length = 5,
                                 .payload = {
                                   offset_bytes[0],
                                   offset_bytes[1],
                                   offset_bytes[2],
                                   offset_bytes[3],
                                   i,
                                 } };
      m_can_transceiver->send(offset_request_reply);
    }
    i++;
  }
  // return state_estimate
  while (true) {
    auto get_request = m_get_chassis_velocities_message_finder.find();
    if (get_request) {
      if (get_request->length == 0) {
        chassis_velocities velocity_estimate =
          p_drivetrain.get_state_estimate();
        auto x_vel_array = int16_to_byte_array(
          float_to_fixed_point_16(velocity_estimate.translation.x, -12));
        auto y_vel_array = int16_to_byte_array(
          float_to_fixed_point_16(velocity_estimate.translation.y, -12));
        auto rot_vel_array = int16_to_byte_array(
          float_to_fixed_point_16(velocity_estimate.rotational_vel, -6));
        hal::can_message reply{ .id = static_cast<uint32_t>(
                                  can_message_id::set_chassis_velocities_reply),
                                .length = 6,
                                .payload = { x_vel_array[0],
                                             x_vel_array[1],
                                             y_vel_array[0],
                                             y_vel_array[1],
                                             rot_vel_array[0],
                                             rot_vel_array[1] } };
        // clear remaining requests
        while (m_get_chassis_velocities_message_finder.find())
          ;
        m_can_transceiver->send(reply);
        break;
      }
    } else {
      break;
    }
  }
}

void mission_control_manager::clear_heartbeat_requests()
{
  while (m_heartbeat_message_finder.find())
    ;
}
void mission_control_manager::reply_heartbeat()
{
  if (m_heartbeat_message_finder.find()) {
    m_can_transceiver->send(
      { .id = static_cast<uint32_t>(can_message_id::heartbeat_reply),
        .length = 0 });
  }
}

}  // namespace sjsu::drive