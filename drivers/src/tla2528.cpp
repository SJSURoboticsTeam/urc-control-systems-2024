#include <tla2528.hpp>
#include <libhal/error.hpp>
#include <libhal-util/i2c.hpp>

namespace sjsu::drivers {

tla2528::tla2528(hal::i2c& p_i2c, hal::steady_clock& p_clk, hal::byte p_i2c_address, float p_analog_supply_voltage) :
    m_bus(p_i2c), m_clk(p_clk)
{
    m_i2c_address = p_i2c_address;
    m_analog_supply_voltage = p_analog_supply_voltage;
}

void tla2528::set_channel(hal::byte p_channel) {
    if (p_channel == m_channel) return;
    if (p_channel > 7) throw hal::argument_out_of_domain(this);

    std::array<hal::byte, 3> cmd_buffer = {
      op_codes::single_register_write,
      register_addresses::channel_sel,
      p_channel
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_pin_mode(pin_mode p_mode) {
    //TODO: add adapter safety
    std::array<hal::byte, 7> cmd_buffer = {
        op_codes::continuous_register_write,
        register_addresses::pin_cfg,
        0x00, 0x00, 0x00, 0x00, 0x00
    };
    if (p_mode != pin_mode::analog_input) {
        cmd_buffer[2] = 0x01;
        if (p_mode != pin_mode::digital_input) {
            cmd_buffer[4] = 0x01;
            if (p_mode != pin_mode::digital_output_open_drain) {
                cmd_buffer[6] = 0x01;
            }
        }
    }
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}


void tla2528::set_digital_out(hal::byte p_channel, bool level) {
    set_channel(p_channel);
    std::array<hal::byte, 3> cmd_buffer = {
      op_codes::single_register_write,
      register_addresses::gpo_value,
      level
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}
bool tla2528::get_digital_out(hal::byte p_channel) {
    set_channel(p_channel);
    std::array<hal::byte, 1> data_buffer;
    std::array<hal::byte, 2> cmd_buffer = {
      op_codes::single_register_read,
      register_addresses::gpo_value,
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
    hal::read(m_bus, m_i2c_address, data_buffer);
    return data_buffer[0];
}

bool tla2528::get_digital_in(hal::byte p_channel) {
    set_channel(p_channel);
    std::array<hal::byte, 1> data_buffer;
    std::array<hal::byte, 3> cmd_buffer = {
      op_codes::single_register_write,
      register_addresses::gpi_value,
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
    hal::read(m_bus, m_i2c_address, data_buffer);
    return data_buffer[0];
}

float tla2528::get_analog_in(hal::byte p_channel){
    set_channel(p_channel);
    std::array<hal::byte, 2> data_buffer;
    std::array<hal::byte, 1> read_cmd_buffer = { op_codes::single_register_read };

    hal::write(m_bus, m_i2c_address, read_cmd_buffer);
    hal::read(m_bus, m_i2c_address, data_buffer);

    uint16_t data = (data_buffer[0] << 4) | (data_buffer[1] >> 4);
    float read_voltage = m_analog_supply_voltage / 4096 * data;
    
    return read_voltage;
}

}  // namespace sjsu::drivers
