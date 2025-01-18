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


  //temp sens 
  std::array<hal::byte, 3> write_buff = { device_addresses::T_CONFIG,
                                          0x01};
  hal::write(m_i2c, device_addresses::i2c_address, write_buff);

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
  int16_t range = 40;
  // int16_t temp = buffer[0] << 8 | buffer[1];
  // ret.temperature = ;

  int16_t x = buffer[2] << 8 | buffer[3];
  ret.x_field = (x * range) / 32768.0;
  int16_t y = buffer[4] << 8 | buffer[5];
  ret.y_field = (y * range) / 32768.0;
  int16_t z = buffer[6] << 8 | buffer[7];
  ret.z_field = (z * range) / 32768.0;
  
  int16_t angle_val = (buffer[9] << 8 | buffer[10]) >> 4;
  float angle_dec = (buffer[10] & 0b0001111) / 16.0;
  ret.angle = angle_val + angle_dec;

  ret.magnitude = buffer[11];

  

  return ret;
}

}  // namespace sjsu::drivers