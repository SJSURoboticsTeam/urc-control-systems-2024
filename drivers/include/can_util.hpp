#pragma once

#include <array>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <sys/types.h>

namespace sjsu::drivers::can_util {

/**
 * @brief converts a fixed point number to floating point
 *
 * @param p_num input fixed point number
 * @param p_exponent 2^p_exponent is equivent to the LSB in fixed point format
 */
constexpr float fixed_to_floating_point_32(hal::i32 p_num,
                                           int const& p_exponent)
{
  float shifted = p_num / powf(2, p_exponent);
  return shifted;
}
/**
 * @brief converts a fixed point number to floating point
 *
 * @param p_num input fixed point number
 * @param p_exponent 2^p_exponent is equivent to the LSB in fixed point format
 */
constexpr float fixed_to_floating_point_16(hal::i16 p_num,
                                           int const& p_exponent)
{
  float shifted = p_num / powf(2, p_exponent);
  return shifted;
}
/**
 * @brief converts a floating point number to fixedpoint
 *
 * @param p_num input floating point number
 * @param p_exponent 2^p_exponent is equivent to the LSB in fixed point format
 */
// endcode message to mission control
constexpr hal::i32 floating_to_fixed_point_32(float const& p_num,
                                              int const& p_exponent)
{
  float initial = p_num * powf(2, p_exponent);
  hal::i32 shifted = static_cast<hal::i32>(initial);
  return (shifted);
}
/**
 * @brief converts a floating point number to fixedpoint
 *
 * @param p_num input floating point number
 * @param p_exponent 2^p_exponent is equivent to the LSB in fixed point format
 */
constexpr hal::i16 floating_to_fixed_point_16(float const& p_num,
                                              int const& p_exponent)
{
  float initial = p_num * powf(2, p_exponent);
  hal::i16 shifted = static_cast<hal::i16>(initial);
  return (shifted);
}
/**
 * @brief breaks integer into a byte array that is big_endian
 *
 * @param p_num input number
 */
constexpr std::array<hal::byte, 4> int32_to_byte_array_big_endian(
  hal::i32 const& p_num)
{
  return { static_cast<hal::byte>((p_num >> 24) & 0xFF),
           static_cast<hal::byte>((p_num >> 16) & 0xFF),
           static_cast<hal::byte>((p_num >> 8) & 0xFF),
           static_cast<hal::byte>((p_num >> 0) & 0xFF) };
}
/**
 * @brief breaks integer into a byte array that is big_endian
 *
 * @param p_num input number
 */
constexpr std::array<hal::byte, 2> int16_to_byte_array_big_endian(
  hal::i16 const& n)
{
  return { static_cast<hal::byte>((n >> 8) & 0xFF),
           static_cast<hal::byte>((n >> 0) & 0xFF)

  };
}
/**
 * @brief turns byte array into signed int
 *
 * @param p_byte_array bytes of integer in big endian
 */
constexpr hal::i32 byte_array_to_int32_big_endian(
  std::array<hal::byte, 4> const& p_byte_array)
{
  hal::i32 number = 0;
  for (uint32_t i = 0; i < p_byte_array.size(); i++) {
    number = (number << 8) | p_byte_array[i];
  }
  return number;
}
/**
 * @brief turns byte array into signed int
 *
 * @param p_byte_array bytes of integer in big endian
 */
constexpr hal::i16 byte_array_to_int16_big_endian(
  std::array<hal::byte, 2> const& p_byte_array)
{
  hal::i16 number = 0;
  for (uint32_t i = 0; i < p_byte_array.size(); i++) {
    number = (number << 8) | p_byte_array[i];
  }
  return number;
}
/**
 * @brief prints CAN message
 *
 * @param p_console serial output
 * @param p_message CAN message to be printed
 */
constexpr void print_can_message(hal::serial& p_console,
                                 hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [",
                  p_message.id,
                  p_message.length);
  for (int i = 0; i < p_message.length; i++) {
    hal::print<256>(p_console, "0x%02X, ", p_message.payload[i]);
  }
  hal::print(p_console, "]\n");
}
}  // namespace sjsu::drivers::can_util
