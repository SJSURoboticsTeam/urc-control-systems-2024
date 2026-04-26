#include <array>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>
#include <optional>
#include <resource_list.hpp>

namespace {
enum class can_message_id : uint32_t
{
  set_target_angle = 0x300,
  imu_accel = 0x301,
  imu_gyro = 0x302,
  imu_mag = 0x303,
  toggle_imu_streams = 0x305,
  servo_position = 0x306,
};
}

namespace sjsu::hub {
mission_control_manager::mission_control_manager(
  hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver)
  : m_can_transceiver(p_can_transceiver)
  , m_gimbal_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::set_target_angle)))
  , m_imu_toggle_message_finder(hal::can_message_finder(
      *m_can_transceiver,
      static_cast<uint32_t>(can_message_id::toggle_imu_streams)))
{
}

std::optional<gimbal_target_request>
mission_control_manager::read_gimbal_target_request()
{
  auto console = resources::console();
  std::optional<hal::can_message> gimbal_request_message;

  while (true) {
    std::optional<hal::can_message> check =
      m_gimbal_message_finder.find();

    if (check) {
      if (check->length == 2) {
        gimbal_request_message = check;
      }
    } else {
      break;
    }
  }

  if (!gimbal_request_message) {
    return std::nullopt;
  }

  auto& payload = gimbal_request_message->payload;

  gimbal_target_request gtr = {
    .x_angle = static_cast<uint8_t>(payload[0]),
    .y_angle = static_cast<uint8_t>(payload[1]),
  };

  hal::print<64>(
    *console, "x_angle = %d, y_angle = %d\n", payload[0], payload[1]);

  return gtr;
}

std::optional<imu_toggle_request>
mission_control_manager::read_imu_toggle_request()
{
  std::optional<hal::can_message> toggle_message;

  while (true) {
    std::optional<hal::can_message> check =
      m_imu_toggle_message_finder.find();

    if (check) {
      if (check->length == 3) {
        toggle_message = check;
      }
    } else {
      break;
    }
  }

  if (!toggle_message) {
    return std::nullopt;
  }

  auto& payload = toggle_message->payload;

  imu_toggle_request itr = {
    .accel_on = payload[0] != 0,
    .gyro_on = payload[1] != 0,
    .mag_on = payload[2] != 0,
  };

  return itr;
}

void mission_control_manager::send_servo_position(uint8_t x_angle,
                                                  uint8_t y_angle)
{
  m_can_transceiver->send(
    { .id = static_cast<uint32_t>(can_message_id::servo_position),
      .length = 2,
      .payload = { x_angle, y_angle } });
}

void mission_control_manager::send_imu_accel(int16_axis accel)
{
  std::array<hal::byte, 8> payload{
    static_cast<uint8_t>(accel.x & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(accel.x) >> 8) & 0xFF),
    static_cast<uint8_t>(accel.y & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(accel.y) >> 8) & 0xFF),
    static_cast<uint8_t>(accel.z & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(accel.z) >> 8) & 0xFF)
  };
  m_can_transceiver->send(
    { .id = static_cast<uint32_t>(can_message_id::imu_accel),
      .length = 6,
      .payload = payload });
}

void mission_control_manager::send_imu_gyro(int16_axis gyro)
{
  std::array<hal::byte, 8> payload{
    static_cast<uint8_t>(gyro.x & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(gyro.x) >> 8) & 0xFF),
    static_cast<uint8_t>(gyro.y & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(gyro.y) >> 8) & 0xFF),
    static_cast<uint8_t>(gyro.z & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(gyro.z) >> 8) & 0xFF)
  };
  m_can_transceiver->send(
    { .id = static_cast<uint32_t>(can_message_id::imu_gyro),
      .length = 6,
      .payload = payload });
}

void mission_control_manager::send_imu_mag(int16_axis mag)
{
  std::array<hal::byte, 8> payload{
    static_cast<uint8_t>(mag.x & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(mag.x) >> 8) & 0xFF),
    static_cast<uint8_t>(mag.y & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(mag.y) >> 8) & 0xFF),
    static_cast<uint8_t>(mag.z & 0xFF),
    static_cast<uint8_t>((static_cast<uint16_t>(mag.z) >> 8) & 0xFF)
  };
  m_can_transceiver->send(
    { .id = static_cast<uint32_t>(can_message_id::imu_mag),
      .length = 6,
      .payload = payload });
}
}  // namespace sjsu::hub