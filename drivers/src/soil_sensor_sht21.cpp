#include "../include/soil_sensor_sht21.hpp"
#include <array>
#include <libhal-util/i2c.hpp>
#include <libhal/units.hpp>
#include <span>

namespace sjsu::drivers {
sht21::sht21(hal::i2c& p_i2c)
  : m_i2c(p_i2c)
{
}

void sht21::soft_reset()
{
  hal::write(m_i2c,
             sht21_i2c_address,
             std::array<hal::byte, 1>{ command::soft_reset_command });
}

bool sht21::is_low_battery()
{
  std::array<hal::byte, 1> response_buffer;
  hal::write_then_read(
    m_i2c,
    sht21_i2c_address,
    std::array<hal::byte, 1>{ command::read_user_register_command },
    response_buffer);
  return (response_buffer[0] & 0b01000000) == 0b01000000;
}

void sht21::set_resolution(resolution p_resolution)
{
  std::array<hal::byte, 1> response_buffer;
  hal::write_then_read(
    m_i2c,
    sht21_i2c_address,
    std::array<hal::byte, 1>{ command::read_user_register_command },
    response_buffer);
  response_buffer[0] = (response_buffer[0] & 0b01111110) | p_resolution;
  hal::write(m_i2c,
             sht21_i2c_address,
             std::array<hal::byte, 2>{ command::write_user_register_command,
                                       response_buffer[0] });
}

void sht21::enable_heater(bool p_enabled)
{
  std::array<hal::byte, 1> response_buffer;
  hal::write_then_read(
    m_i2c,
    sht21_i2c_address,
    std::array<hal::byte, 1>{ command::read_user_register_command },
    response_buffer);
  if (p_enabled) {
    response_buffer[0] = (response_buffer[0] & 0b11111011) | 0b00000100;
  } else {
    response_buffer[0] = (response_buffer[0] & 0b11111011) | 0b00000000;
  }
  hal::write(m_i2c,
             sht21_i2c_address,
             std::array<hal::byte, 2>{ command::write_user_register_command,
                                       response_buffer[0] });
}

double sht21::get_relative_humidity()
{
  std::array<hal::byte, 3> response_buffer;

  hal::write_then_read(m_i2c,
                       sht21_i2c_address,
                       std::array<hal::byte, 1>{
                         command::hold_relative_humidity_measurement_command },
                       response_buffer,
                       hal::never_timeout());

  uint16_t raw_relative_humidty =
    (((hal::byte)response_buffer[0]) << 8) | (response_buffer[1] & 0b11111100);

  return -6 + (125.0 / 0x10000) * raw_relative_humidty;
}

double sht21::get_temperature()
{
  std::array<hal::byte, 3> response_buffer;

  hal::write_then_read(
    m_i2c,
    sht21_i2c_address,
    std::array<hal::byte, 1>{ command::hold_temperature_measurement_command },
    response_buffer,
    hal::never_timeout());

  uint16_t raw_temperature =
    (((hal::byte)response_buffer[0]) << 8) | (response_buffer[1] & 0b11111100);

  return -46.85 + (175.72 / 0x10000) * raw_temperature;
}

}  // namespace sjsu::drivers
