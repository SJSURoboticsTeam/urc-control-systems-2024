#include <tla2528.hpp>
#include <libhal/error.hpp>

namespace sjsu::drivers {

tla2528::tla2528(hal::i2c& p_i2c, hal::steady_clock& p_clk, hal::byte p_i2c_address) :
    m_bus(p_i2c), m_clk(p_clk)
{
    m_i2c_address = p_i2c_address;
}

void tla2528::set_channel(hal::byte channel) {
    if (channel == (m_mode & 0b111)) return;
    if (channel > 7) throw hal::argument_out_of_domain(this);

    std::array<hal::byte, 3> cmd_buffer = {
      OpCodes::SingleRegisterWrite,    // Command to write data to a register
      RegisterAddresses::CHANNEL_SEL,  // Register to select channel
      channel
    };
    hal::write(m_bus, m_i2c_address, cmd_buffer);
}

void tla2528::set_pin_mode(PinMode p_mode) {
    std::array<hal::byte, 3> cmd_buffer = {
        OpCodes::SingleRegisterWrite,    // Command to write data to a register
        RegisterAddresses::PIN_CFG,  // Register to select channel
        0x00
    };

    
    if (p_mode == PinMode::AnalogInput) {
        //set to analog if digital
        if (m_mode & 0x80) {
            hal::write(m_bus, m_i2c_address, cmd_buffer);
        }
        return;
    }
    //set to digital if analog
    if (!(m_mode & 0x80)) {
        cmd_buffer[2] = 0x01;
        hal::write(m_bus, m_i2c_address, cmd_buffer);
    }

    if (p_mode == PinMode::DigitalInput) {
        //set to digital input if digital output
        if (m_mode & 0x40) {
            cmd_buffer[1] = RegisterAddresses::GPIO_CFG;
            cmd_buffer[2] = 0x00;
            hal::write(m_bus, m_i2c_address, cmd_buffer);
        }
        return;
    }
    //set to digital out if digital input
    if (!(m_mode & 0x40)) {
        cmd_buffer[1] = RegisterAddresses::GPIO_CFG;
        cmd_buffer[2] = 0x01;
        hal::write(m_bus, m_i2c_address, cmd_buffer);
    }
    
    if (p_mode == PinMode::DigitalOutputOpenDrain) {
        //set too open drain if push pull
        if (m_mode & 0x20) {
            cmd_buffer[1] = RegisterAddresses::GPO_DRIVE_CFG;
            cmd_buffer[2] = 0x00;
            hal::write(m_bus, m_i2c_address, cmd_buffer);
        }
        return;
    }
    //set to push pull if open drain
    if (!(m_mode & 0x20)) {
        cmd_buffer[1] = RegisterAddresses::GPIO_CFG;
        cmd_buffer[2] = 0x01;
        hal::write(m_bus, m_i2c_address, cmd_buffer);
    }

}

}  // namespace sjsu::drivers
