#include "../include/apds9960.hpp"
#include <libhal/timeout.hpp>

namespace sjsu::drivers {
apds9960::apds9960(hal::v5::strong_ptr<hal::i2c> p_i2c,
                   hal::v5::strong_ptr<hal::steady_clock> p_clock,
                   hal::v5::strong_ptr<hal::serial> p_terminal)
  : m_i2c(p_i2c)
  , m_clock(p_clock)
  , m_terminal(p_terminal)
{
  default_enable();
}

void apds9960::default_enable()
{
  // hal::byte config1_data = 0x60;  // 01100000
  // std::array<hal::byte, 2> write_buff = { register_set::config1, config1_data
  // }; hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  // hal::delay(*m_clock, 10ms);

  // hal::byte config2_data = 0x01;  // 00000001
  // write_buff = { register_set::config2, config2_data };
  // hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());

  // hal::delay(*m_clock, 10ms);

  hal::byte aen_enable = 0x03;  // 00000011
  std::array<hal::byte, 2> write_buff = { register_set::enable, aen_enable };
  hal::write(*m_i2c, m_address, write_buff, hal::never_timeout());
  hal::delay(*m_clock, 10ms);

  // hal::delay(*m_clock, 10ms);

  // hal::byte control_reg = 0x02;  // 00000010 (16x)
  // write_buff = { register_set::control, control_reg };
  // control_reg = write_buff[0];
}

apds9960::color apds9960::readColor()
{
  color data_color;
  hal::print<64>(*m_terminal, "Reading from color sensor\n");

  std::array<hal::byte, 1> data_buff_low;
  std::array<hal::byte, 1> data_buff_high;

  std::array<hal::byte, 1> write_to_address = { register_set::cdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_low);
  hal::delay(*m_clock, 10ms);
  write_to_address = { register_set::cdatah };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_high);

  hal::print<64>(*m_terminal, "low byte: %u\n", data_buff_low[0]);
  hal::print<64>(*m_terminal, "high byte: %u\n", data_buff_high[0]);

  data_color.clear_data = data_buff_high[0] << 8 | data_buff_low[0];

  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::rdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_low);
  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::rdatah };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_high);

  hal::print<64>(*m_terminal, "Red Low Byte: %u\n", data_buff_low[0]);
  hal::print<64>(*m_terminal, "Red High Byte: %u\n", data_buff_high[0]);

  data_color.red_data = data_buff_high[0] << 8 | data_buff_low[0];

  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::gdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_low);
  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::gdatah };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_high);

  hal::print<64>(*m_terminal, "Green Low Byte: %u\n", data_buff_low[0]);
  hal::print<64>(*m_terminal, "Green High Byte: %u\n", data_buff_high[0]);

  data_color.green_data = data_buff_high[0] << 8 | data_buff_low[0];

  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::bdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_low);
  hal::delay(*m_clock, 10ms);

  write_to_address = { register_set::bdatah };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff_high);

  hal::print<64>(*m_terminal, "Blue Low Byte: %u\n", data_buff_low[0]);
  hal::print<64>(*m_terminal, "Blue High Hyte: %u\n", data_buff_high[0]);

  data_color.blue_data = data_buff_high[0] << 8 | data_buff_low[0];

  hal::delay(*m_clock, 10ms);

  return data_color;

  // std::array<hal::byte, 2> data_buff;

  // std::array<hal::byte, 1> write_to_address = { register_set::cdatal };
  // hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  // hal::print<64>(*m_terminal, "low byte: %u\n", data_buff[0]);
  // hal::print<64>(*m_terminal, "high byte: %u\n", data_buff[1]);
  // data_color.clear_data = data_buff[1] << 8 | data_buff[0];

  // write_to_address = { register_set::rdatal };
  // hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  // hal::print<64>(*m_terminal, "low byte: %u\n", data_buff[0]);
  // hal::print<64>(*m_terminal, "high byte: %u\n", data_buff[1]);
  // data_color.red_data = data_buff[1] << 8 | data_buff[0];

  // write_to_address = { register_set::gdatal };
  // hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  // hal::print<64>(*m_terminal, "low byte: %u\n", data_buff[0]);
  // hal::print<64>(*m_terminal, "high byte: %u\n", data_buff[1]);
  // data_color.green_data = data_buff[1] << 8 | data_buff[0];

  // write_to_address = { register_set::bdatal };
  // hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  // hal::print<64>(*m_terminal, "low byte: %u\n", data_buff[0]);
  // hal::print<64>(*m_terminal, "high byte: %u\n", data_buff[1]);
  // data_color.blue_data = data_buff[1] << 8 | data_buff[0];

  // return data_color;
}
}  // namespace sjsu::drivers