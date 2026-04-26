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
  /**
   * @param p_can_transceiver CAN transceiver used to send and receive messages
   */
  mission_control_manager(
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);

  /**
   * @brief reads the most recent gimbal target command (0x300) from the CAN
   * buffer, discarding any older messages
   *
   * @return the most recent gimbal target with x and y angles (0-180), or
   * std::nullopt if no command was received
   */
  std::optional<gimbal_target_request> read_gimbal_target_request();

  /**
   * @brief reads the most recent IMU toggle command (0x305) from the CAN
   * buffer, discarding any older messages
   *
   * @return which IMU streams to enable or disable, or std::nullopt if no
   * toggle command was received
   */
  std::optional<imu_toggle_request> read_imu_toggle_request();

  /**
   * @brief sends the current servo position over CAN (0x306). Sent
   * periodically to act as a heartbeat for mission control
   *
   * @param x_angle current x servo angle (0-180)
   * @param y_angle current y servo angle (0-180)
   */
  void send_servo_position(uint8_t x_angle, uint8_t y_angle);

  /**
   * @brief sends accelerometer data over CAN (0x301). Only called when
   * accelerometer stream is toggled on via 0x305
   *
   * @param accel 3-axis accelerometer data as 16-bit signed integers
   */
  void send_imu_accel(int16_axis accel);

  /**
   * @brief sends gyroscope data over CAN (0x302). Only called when gyroscope
   * stream is toggled on via 0x305
   *
   * @param gyro 3-axis gyroscope data as 16-bit signed integers
   */
  void send_imu_gyro(int16_axis gyro);

  /**
   * @brief sends magnetometer data over CAN (0x303). Only called when
   * magnetometer stream is toggled on via 0x305
   *
   * @param mag 3-axis magnetometer data as 16-bit signed integers
   */
  void send_imu_mag(int16_axis mag);

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::can_message_finder m_gimbal_message_finder;
  hal::can_message_finder m_imu_toggle_message_finder;
};
}  // namespace sjsu::hub