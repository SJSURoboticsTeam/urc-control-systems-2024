#include <tla2528.hpp>
#include <libhal/error.hpp>

namespace sjsu::drivers {

tla2528::tla2528(hal::i2c& p_i2c, hal::steady_clock& p_clk, hal::byte p_i2c_address) :
    m_bus(p_i2c), m_clk(p_clk)
{
    m_i2c_address = p_i2c_address;
}

void tla2528::set_channel(hal::byte p_channel) {
    if (p_channel == m_channel) return;
    if (p_channel > 7) throw hal::argument_out_of_domain(this);

    std::array<hal::byte, 3> cmd_buffer = {
      OpCodes::SingleRegisterWrite,    // Command to write data to a register
      RegisterAddresses::CHANNEL_SEL,  // Register to select channel
      p_channel
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_pin_mode(PinMode p_mode) {
    std::array<hal::byte, 5> cmd_buffer = {
        OpCodes::ContinuousRegisterWrite,    // Command to write data to registers
        RegisterAddresses::PIN_CFG,  // Starting config register
        0x00, 0x00, 0x00
    };
    if (p_mode != PinMode::AnalogInput) {
        cmd_buffer[2] = 0x01;
        if (p_mode != PinMode::DigitalInput) {
            cmd_buffer[3] = 0x01;
            if (p_mode != PinMode::DigitalOutputOpenDrain) {
                cmd_buffer[4] = 0x01;
            }
        }
    }
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}

}  // namespace sjsu::drivers
