#include "soil_sensor_sht21.hpp"
#include <array>
#include <span>
#include <libhal/units.hpp>

hal::result<hal::sht::sht21> hal::sht::sht21::create(hal::i2c& p_bus, hal::byte p_address) {
  hal::sht::sht21 sensor(p_bus, p_address);

  HAL_CHECK(sensor.soft_reset());

  return sensor;
}
      
hal::status hal::sht::sht21::soft_reset() {
  HAL_CHECK(hal::write(*m_i2c, m_address, std::array<hal::byte, 1> { command::soft_reset_command }));
  return hal::success();
}

hal::result<bool> hal::sht::sht21::is_low_battery() {
  std::array<hal::byte, 1> response_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, std::array<hal::byte, 1> { command::read_user_register_command }, response_buffer));
  return (response_buffer[0] & 0b01000000) == 0b01000000;
}

hal::status hal::sht::sht21::set_resolution(resolution p_resolution) {
  std::array<hal::byte, 1> response_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, std::array<hal::byte, 1> { command::read_user_register_command }, response_buffer));
  response_buffer[0] = (response_buffer[0] & 0b01111110) | p_resolution;
  HAL_CHECK(hal::write(*m_i2c, m_address, std::array<hal::byte, 2> { command::write_user_register_command, response_buffer[0] } ));
  return hal::success();
}

hal::status hal::sht::sht21::enable_heater(bool p_enabled) {
  std::array<hal::byte, 1> response_buffer;
  HAL_CHECK(hal::write_then_read(*m_i2c, m_address, std::array<hal::byte, 1> { command::read_user_register_command }, response_buffer));
  if(p_enabled) {
    response_buffer[0] = (response_buffer[0] & 0b11111011) | 0b00000100;
  }else {
    response_buffer[0] = (response_buffer[0] & 0b11111011) | 0b00000000;
  }
  HAL_CHECK(hal::write(*m_i2c, m_address, std::array<hal::byte, 2> { command::write_user_register_command, response_buffer[0] } ));
  return hal::success();
}

hal::result<double> hal::sht::sht21::get_relative_humidity() {
  std::array<hal::byte, 3> response_buffer;

  HAL_CHECK(hal::write_then_read(
    *m_i2c, 
    m_address, 
    std::array<hal::byte, 1> { command::hold_relative_humidity_measurement_command }, 
    response_buffer, 
    hal::never_timeout()));
  
  uint16_t raw_relative_humidty = (((hal::byte) response_buffer[0]) << 8) | (response_buffer[1] & 0b11111100);

  return -6 + (125.0 / 0x10000) * raw_relative_humidty; 
}


hal::result<double> hal::sht::sht21::get_temperature() {
  std::array<hal::byte, 3> response_buffer;

  HAL_CHECK(hal::write_then_read(
    *m_i2c, 
    m_address, 
    std::array<hal::byte, 1> { command::hold_temperature_measurement_command }, 
    response_buffer, 
    hal::never_timeout()));
  
  uint16_t raw_temperature = (((hal::byte) response_buffer[0]) << 8) | (response_buffer[1] & 0b11111100);

  return -46.85 + (175.72 / 0x10000) * raw_temperature; 
}


hal::sht::sht21::sht21(hal::i2c& p_bus, hal::byte p_address) {
  m_i2c = &p_bus;
  m_address = p_address;
}
