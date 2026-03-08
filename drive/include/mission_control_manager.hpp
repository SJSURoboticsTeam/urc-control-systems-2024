#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <drivetrain.hpp>
#include <libhal-util/can.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <optional>
#include <swerve_module.hpp>

namespace sjsu::drive {

struct chassis_velocities_request
{
  chassis_velocities chassis_vels;
  /**
   * MC uses it as if you should resolve_module conflicts
   * Drive uses it as if it ran into a module conflict
   */
  bool module_conflicts;
};

class mission_control_manager
{
public:
  /**
   * @brief mission_control_manager abstracts much of the message serialization
   * for the user
   *
   * @param p_can_transceiver can bus used from communication with mission
   * control
   */
  mission_control_manager(
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver);
  /**
   * @brief converts a fixed point value into a float
   * @param p_fixed_point_num value being converted
   * @param p_expo fixed point exponent, i.e value of fixed point value of 1
   * corelates to 2^p_expo in floating point
   * @returns fixed point values converted to float
   */
    /**
   * @brief converts a fixed point value into a float
   * @param p_fixed_point_num value being converted
   * @param p_expo fixed point exponent, i.e value of fixed point value of 1
   * corelates to 2^p_expo in floating point
   * @returns fixed point values converted to float
   */
  float fixed_point_16_to_float(int16_t p_fixed_point_num, int p_expo);
  /**
   * @brief converts a float into a fixed point value
   * @param p_float_num value being converted
   * @param p_expo fixed point exponent, i.e value of fixed point=1 corelates to
   * 2^p_expo in floating point
   * @returns floating point value converted to fixed point
   */
  int16_t float_to_fixed_point_16(float p_float_num, int p_expo);
  /**
   * @brief gets int16_t from a 2 byte array
   * @param p_array byte array of values put in big endian
   * @returns int value in given array
   */
  int16_t byte_array_to_int16(std::array<hal::byte, 2> p_array);
  /**
   * @brief puts int16_t into a 2 byte array
   * @param p_num int value
   * @returns byte array of the input in big endian
   */
  std::array<hal::byte, 2> int16_to_byte_array(int16_t p_num);

  float fixed_point_32_to_float(int32_t p_fixed_point_num, int p_expo);
  /**
   * @brief converts a float into a fixed point value
   * @param p_float_num value being converted
   * @param p_expo fixed point exponent, i.e value of fixed point=1 corelates to
   * 2^p_expo in floating point
   * @returns floating point value converted to fixed point
   */
  int32_t float_to_fixed_point_32(float p_float_num, int p_expo);
  /**
   * @brief gets int32_t from a 4 byte array
   * @param p_array byte array of values put in big endian
   * @returns int value in given array
   */
  int32_t byte_array_to_int32(std::array<hal::byte, 4> p_array);
  /**
   * @brief puts int32_t into a 4 byte array
   * @param p_num int value
   * @returns byte array of the input in big endian
   */
  std::array<hal::byte, 4> int32_to_byte_array(int32_t p_num);

  /**
   * @brief returns most recent target velocity request from mission control
   *
   * @returns most recent target velocity request
   */
  std::optional<chassis_velocities_request> read_set_velocity_request();
  void reply_set_velocity_request(
    chassis_velocities_request const& p_chassis_vel);

  /**
   * @brief if homing was requested
   * @returns true if homing requested
   */
  bool read_homing_request();
  /**
   * @brief sends a reply to homing requests
   */
  void reply_homing_request();
  /**
   * @brief replys to all data requests from mission control
   * @param p_drivetrain drivetrain reference used to read data
   */
  void fulfill_data_requests(drivetrain const& p_drivetrain);

  /**
   * @brief clear's heartbeat request queue
   */
  void clear_heartbeat_requests();
  /**
   * @brief responds to heartbeat request
   */
  void reply_heartbeat();

  // TODO: Config & Config reply

private:
  hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
  hal::can_message_finder m_heartbeat_message_finder;
  hal::can_message_finder m_set_chassis_velocities_message_finder;
  hal::can_message_finder m_get_chassis_velocities_message_finder;
  hal::can_message_finder m_homing_request_message_finder;
  hal::can_message_finder m_get_offset_message_finder;
};

}  // namespace sjsu::drive