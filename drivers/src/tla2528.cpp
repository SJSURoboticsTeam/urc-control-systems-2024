#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <tla2528.hpp>

namespace sjsu::drivers {

tla2528::tla2528(hal::i2c& p_i2c, hal::byte p_i2c_address)
  : m_bus(p_i2c)
{
  m_i2c_address = p_i2c_address;
  // TODO: reset command
}

void tla2528::set_analog_channel(hal::byte p_channel)
{
  if (p_channel == m_channel)
    return;
  if (p_channel > 7)
    throw hal::argument_out_of_domain(this);

  std::array<hal::byte, 3> cmd_buffer = { op_codes::single_register_write,
                                          register_addresses::channel_sel,
                                          p_channel };
  hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_pin_mode(pin_mode p_mode, hal::byte p_channel)
{
  if (p_channel > 7)
    throw hal::argument_out_of_domain(this);
  hal::bit_mask channel_mask = hal::bit_mask::from(p_channel);
  if (p_mode == pin_mode::digital_output_push_pull ||
      p_mode == pin_mode::digital_output_open_drain) {
    if (hal::bit_extract(channel_mask, m_object_created) &&
        !(hal::bit_extract(channel_mask, m_pin_cfg) ||
          hal::bit_extract(channel_mask, m_gpio_cfg))) {
      throw hal::resource_unavailable_try_again(this);
    }
    hal::bit_modify(m_pin_cfg).set(channel_mask);
    hal::bit_modify(m_gpio_cfg).set(channel_mask);
    if (p_mode == pin_mode::digital_output_push_pull) {
      hal::bit_modify(m_gpo_drive_cfg).set(channel_mask);
    } else {
      hal::bit_modify(m_gpo_drive_cfg).clear(channel_mask);
    }
  } else if (hal::bit_extract(channel_mask, m_object_created)) {
    throw hal::resource_unavailable_try_again(this);
  } else if (p_mode == pin_mode::analog_input) {
    hal::bit_modify(m_pin_cfg).clear(channel_mask);
  } else {  // must be pin_mode::digitalInput
    hal::bit_modify(m_pin_cfg).set(channel_mask);
    hal::bit_modify(m_gpio_cfg).clear(channel_mask);
  }
  std::array<hal::byte, 7> cmd_buffer = { op_codes::continuous_register_write,
                                          register_addresses::pin_cfg,
                                          m_pin_cfg,
                                          0x00,
                                          m_gpio_cfg,
                                          0x00,
                                          m_gpo_drive_cfg };
  hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_digital_out(hal::byte p_values)
{
  m_gpo_value = p_values;
  std::array<hal::byte, 3> cmd_buffer = { op_codes::single_register_write,
                                          register_addresses::gpo_value,
                                          m_gpo_value };
  hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_digital_out(hal::byte p_channel, bool level)
{
  if (p_channel > 7)
    throw hal::argument_out_of_domain(this);
  if (level) {
    hal::bit_modify(m_gpo_value).set(hal::bit_mask::from(p_channel));
  } else {
    hal::bit_modify(m_gpo_value).clear(hal::bit_mask::from(p_channel));
  }
  set_digital_out(m_gpo_value);
}

bool tla2528::get_digital_out(hal::byte p_channel)
{
  if (p_channel > 7)
    throw hal::argument_out_of_domain(this);
  std::array<hal::byte, 1> data_buffer;
  std::array<hal::byte, 2> cmd_buffer = {
    op_codes::single_register_read,
    register_addresses::gpo_value,
  };
  hal::write(m_bus, m_i2c_address, cmd_buffer);
  hal::read(m_bus, m_i2c_address, data_buffer);
  return hal::bit_extract(hal::bit_mask::from(p_channel), data_buffer[0]);
}

hal::byte tla2528::get_digital_in()
{
  std::array<hal::byte, 1> data_buffer;
  std::array<hal::byte, 3> cmd_buffer = {
    op_codes::single_register_read,
    register_addresses::gpi_value,
  };
  hal::write(m_bus, m_i2c_address, cmd_buffer);
  hal::read(m_bus, m_i2c_address, data_buffer);
  return data_buffer[0];
}

bool tla2528::get_digital_in(hal::byte p_channel)
{
  if (p_channel > 7)
    throw hal::argument_out_of_domain(this);
  return hal::bit_extract(hal::bit_mask::from(p_channel), get_digital_in());
}

float tla2528::get_analog_in(hal::byte p_channel)
{
  set_analog_channel(p_channel);
  std::array<hal::byte, 2> data_buffer;
  std::array<hal::byte, 1> read_cmd_buffer = { op_codes::single_register_read };

  hal::write(m_bus, m_i2c_address, read_cmd_buffer);
  hal::read(m_bus, m_i2c_address, data_buffer);

  // TODO: look into averaging & CRC
  uint16_t data = 0;
  hal::bit_modify(data).insert<hal::bit_mask::from(4, 11)>(data_buffer[0]);
  hal::bit_modify(data).insert<hal::bit_mask::from(0, 3)>(
    hal::bit_extract(hal::bit_mask::from(4, 7), data_buffer[1]));
  return data / 4095.0;
}

}  // namespace sjsu::drivers
