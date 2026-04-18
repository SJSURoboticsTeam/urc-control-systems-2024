#pragma once

#include <array>
#include <cstdint>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <optional>

namespace sjsu::hub {
struct int16_axis
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct gimbal_target_request
{
  uint8_t x_angle;
  uint8_t y_angle;
};

struct imu_toggle_request
{
  bool accel_on;
  bool gyro_on;
  bool mag_on;
};

class mission_control_manager
{
public:
  mission_control_manager(
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);

  std::optional<gimbal_target_request> read_gimbal_target_request();
  std::optional<imu_toggle_request> read_imu_toggle_request();

  void send_servo_position(uint8_t x_angle, uint8_t y_angle);

  void send_imu_accel(int16_axis accel);
  void send_imu_gyro(int16_axis gyro);
  void send_imu_mag(int16_axis mag);

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::can_message_finder m_gimbal_message_finder;
  hal::can_message_finder m_imu_toggle_message_finder;
};
}  // namespace sjsu::hub