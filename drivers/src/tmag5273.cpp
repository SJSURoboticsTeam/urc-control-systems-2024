#include "../include/tmag5273.hpp"
using namespace std::chrono_literals;

namespace sjsu::drivers {

tmag5273::tmag5273(hal::i2c& p_i2c, hal::steady_clock& p_clock)
  : m_i2c(p_i2c)
  , m_clock(p_clock)
{
}

void tmag5273::defualt_config()
{
  hal::byte dev_config_1 = 0x0C; //00001100
  hal::byte dev_config_2 = 0x0A;//00001010
  tmag5273::config_device(dev_config_1, dev_config_2);

  hal::byte sens_config_1 = 0x70; //01110000
  hal::byte sens_config_2 = 0x08; // 00001000

  tmag5273::config_sensor(sens_config_1, sens_config_2);
}

void tmag5273::config_device(hal::byte config_1, hal::byte config_2)
{
  std::array<hal::byte, 3> write_buff = { device_addresses::DEVICE_CONFIG_1,
                                          config_1,
                                          config_2 };
hal::
  write(m_i2c, device_addresses::i2c_address, write_buff);
}

void tmag5273::config_sensor(hal::byte config_1, hal::byte config_2)
{
  std::array<hal::byte, 3> write_buff = { device_addresses::SENSOR_CONFIG_1,
                                          config_1, config_2
 };
hal::
  write(m_i2c, device_addresses::i2c_address, write_buff);
}

tmag5273::data tmag5273::read()
{
  tmag5273::data ret;
  std::array<hal::byte, 12> buffer;
  std::array<hal::byte, 1> adresses_write_to = {
    device_addresses::T_MSB_RESULT
  };
  hal::write_then_read(
    m_i2c, device_addresses::i2c_address, adresses_write_to, buffer);
  ret.temperature = buffer[0] << 8 | buffer[1];
  ret.x_field = buffer[2] << 8 | buffer[3];
  ret.y_field = buffer[4] << 8 | buffer[5];
  ret.z_field = buffer[6] << 8 | buffer[7];

  adresses_write_to = {
    device_addresses::ANGLE_RESULT_MSB
  };
  hal::write_then_read(
    m_i2c, device_addresses::i2c_address, adresses_write_to, buffer);
  ret.angle = buffer[0] << 8 | buffer[1];
  ret.magnitude = buffer[2] << 8 | buffer[3];

  return ret;
}

}  // namespace sjsu::drivers