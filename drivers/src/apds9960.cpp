#include "../include/apds9960.hpp"

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
  hal::byte aen_enable = 0x03;  // 00000011
  std::array<hal::byte, 2> write_buff = { register_set::enable, aen_enable };
  hal::write(*m_i2c, m_address, write_buff);
}

apds9960::color apds9960::readColor()
{
  color data_color;

  std::array<hal::byte, 2> data_buff;

  std::array<hal::byte, 1> write_to_address = { register_set::cdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  data_color.clear_data = data_buff[1] << 8 | data_buff[0];

  write_to_address = { register_set::rdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  data_color.red_data = data_buff[1] << 8 | data_buff[0];

  write_to_address = { register_set::gdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  data_color.green_data = data_buff[1] << 8 | data_buff[0];

  write_to_address = { register_set::bdatal };
  hal::write_then_read(*m_i2c, m_address, write_to_address, data_buff);
  data_color.blue_data = data_buff[1] << 8 | data_buff[0];

  return data_color;
}
}  // namespace sjsu::drivers