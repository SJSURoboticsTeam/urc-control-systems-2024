#include <bitset>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <tla2528.hpp>

namespace {
enum op_codes : hal::byte
{
  single_register_read = 0b0001'0000,
  single_register_write = 0b0000'1000,
  set_bit = 0b0001'1000,
  clear_bit = 0b0010'0000,
  // Continuously reads data from a group of registers. Provide the first
  // address to read from, if it runs out of valid addresses to read, it
  // returns zeros. (See Figure 30 on datasheet)
  continuous_register_read = 0b0011'0000,
  // Continuously writes data to a group of registers. Provide the first
  // address to write to.The data sent will automatically write the data to
  // the next register in ascending order. (See Figure 32 on datasheet)
  continuous_register_write = 0b0010'1000
};

enum register_addresses : hal::byte
{
  system_status = 0x0,
  general_cfg = 0x1,
  data_cfg = 0x2,
  osr_cfg = 0x3,
  opmode_cfg = 0x4,
  pin_cfg = 0x5,
  gpio_cfg = 0x7,
  gpo_drive_cfg = 0x9,
  gpo_value = 0xB,
  gpi_value = 0xD,
  sequence_cfg = 0x10,
  channel_sel = 0x11,
  auto_seq_ch_sel = 0x12
};
}  // namespace
namespace sjsu::drivers {

tla2528::tla2528(hal::i2c& p_i2c, hal::byte p_i2c_address)
  : m_i2c_bus(p_i2c)
  , m_i2c_address(p_i2c_address)
{
  // TODO: reset command
}

void tla2528::set_analog_channel(hal::byte p_channel)
{
  throw_if_invalid_channel(p_channel);
  if (p_channel == m_channel) {
    return;
  }
  std::array<hal::byte, 3> cmd_buffer = { op_codes::single_register_write,
                                          register_addresses::channel_sel,
                                          p_channel };
  hal::write(m_i2c_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_pin_mode(pin_mode p_mode, hal::byte p_channel)
{
  throw_if_invalid_channel(p_channel);
  std::array<hal::byte, 5> data_buffer;
  std::array<hal::byte, 2> read_cmd_buffer = {
    op_codes::continuous_register_read, register_addresses::pin_cfg
  };
  hal::write_then_read(m_i2c_bus, m_i2c_address, read_cmd_buffer, data_buffer);

  hal::byte pin_cfg_reg = data_buffer[0];
  hal::byte gpio_cfg_reg = data_buffer[2];
  hal::byte gpo_drive_cfg_reg = data_buffer[4];

  hal::bit_mask channel_mask = hal::bit_mask::from(p_channel);
  if (p_mode == pin_mode::digital_output_push_pull ||
      p_mode == pin_mode::digital_output_open_drain) {
    if (hal::bit_extract(channel_mask, m_object_created) &&
        !(hal::bit_extract(channel_mask, pin_cfg_reg) ||
          hal::bit_extract(channel_mask, gpio_cfg_reg))) {
      throw hal::resource_unavailable_try_again(this);
    }
    hal::bit_modify(pin_cfg_reg).set(channel_mask);
    hal::bit_modify(gpio_cfg_reg).set(channel_mask);
    if (p_mode == pin_mode::digital_output_push_pull) {
      hal::bit_modify(gpo_drive_cfg_reg).set(channel_mask);
    } else {
      hal::bit_modify(gpo_drive_cfg_reg).clear(channel_mask);
    }
  } else if (hal::bit_extract(channel_mask, m_object_created)) {
    throw hal::resource_unavailable_try_again(this);
  } else if (p_mode == pin_mode::analog_input) {
    hal::bit_modify(pin_cfg_reg).clear(channel_mask);
  } else {  // must be pin_mode::digitalInput
    hal::bit_modify(pin_cfg_reg).set(channel_mask);
    hal::bit_modify(gpio_cfg_reg).clear(channel_mask);
  }
  std::array<hal::byte, 7> write_cmd_buffer = {
    op_codes::continuous_register_write,
    register_addresses::pin_cfg,
    pin_cfg_reg,
    0x00,
    gpio_cfg_reg,
    0x00,
    gpo_drive_cfg_reg
  };
  hal::write(m_i2c_bus, m_i2c_address, write_cmd_buffer);
}

void tla2528::set_digital_bus_out(hal::byte p_values)
{
  m_gpo_value = p_values;
  std::array<hal::byte, 3> cmd_buffer = { op_codes::single_register_write,
                                          register_addresses::gpo_value,
                                          m_gpo_value };
  hal::write(m_i2c_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_digital_out(hal::byte p_channel, bool p_high)
{
  throw_if_invalid_channel(p_channel);
  if (p_high) {
    hal::bit_modify(m_gpo_value).set(hal::bit_mask::from(p_channel));
  } else {
    hal::bit_modify(m_gpo_value).clear(hal::bit_mask::from(p_channel));
  }
  set_digital_bus_out(m_gpo_value);
}

bool tla2528::get_digital_out(hal::byte p_channel)
{
  throw_if_invalid_channel(p_channel);
  return hal::bit_extract(hal::bit_mask::from(p_channel),
                          get_digital_bus_out());
}
hal::byte tla2528::get_digital_bus_out()
{
  std::array<hal::byte, 1> data_buffer;
  std::array<hal::byte, 2> cmd_buffer = {
    op_codes::single_register_read,
    register_addresses::gpo_value,
  };
  hal::write_then_read(m_i2c_bus, m_i2c_address, cmd_buffer, data_buffer);
  return data_buffer[0];
}

hal::byte tla2528::get_digital_bus_in()
{
  std::array<hal::byte, 1> data_buffer;
  std::array<hal::byte, 3> cmd_buffer = {
    op_codes::single_register_read,
    register_addresses::gpi_value,
  };
  hal::write_then_read(m_i2c_bus, m_i2c_address, cmd_buffer, data_buffer);
  return data_buffer[0];
}

bool tla2528::get_digital_in(hal::byte p_channel)
{
  throw_if_invalid_channel(p_channel);
  return hal::bit_extract(hal::bit_mask::from(p_channel), get_digital_bus_in());
}

float tla2528::get_analog_in(hal::byte p_channel)
{
  set_analog_channel(p_channel);
  // TODO: look into averaging & channel validation
  std::array<hal::byte, 2> data_buffer;
  std::array<hal::byte, 1> cmd_buffer = { op_codes::single_register_read };
  hal::write_then_read(m_i2c_bus, m_i2c_address, cmd_buffer, data_buffer);

  // Take 12 bit number stored in first 12 bits of 2 bytes and converting to 16
  // bit num by shifting 4 bit right (See Figure 25 on datasheet)
  uint16_t data = data_buffer[0] << 4 | data_buffer[1] >> 4;
  return data / 4095.0;
}

void tla2528::throw_if_invalid_channel(hal::byte p_channel)
{
  if (p_channel > 7) {
    throw hal::argument_out_of_domain(this);
  }
}

}  // namespace sjsu::drivers
