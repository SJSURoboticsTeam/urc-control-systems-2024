#include "../include/adc128d818.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
adc128d818::adc128d818(hal::v5::strong_ptr<hal::i2c> p_i2c,
           hal::v5::strong_ptr<hal::steady_clock> p_clock,
           hal::v5::strong_ptr<hal::serial> p_terminal)
           : m_i2c(p_i2c)
           , m_clock(p_clock)
           , m_terminal(p_terminal)
{
    startup_sequence();
}

void adc128d818::startup_sequence()
{   
    hal::delay(*m_clock, 33ms);

    //wait for busy_status register to clear
    while(true)
    {
        std::array<hal::byte,1> status = {0};
        read_register(register_addresses::busy_status, status);
        if(((status[0]) & bit_masks::busy) == 0) break;
    }

    write_register(register_addresses::adv_config, bit_masks::mode_0); //programming advanced configuration register
    write_register(register_addresses::conv_rate, bit_masks::cont_conv); //programming conversion rate register
    write_register(register_addresses::ch_enable, bit_masks::enable_ch); //enabling channels: IN0, IN1, IN4, IN5, IN7 <- change when pin connections change
    write_register(register_addresses::config, bit_masks::start); //setting START bit
}

float adc128d818::read_voltage()
{
    std::array<hal::byte,2> raw = {0};
    read_16bit_register(register_addresses::in0, raw);
    uint16_t code16 = (raw[0] << 8) | raw[1];
    uint16_t code12 = code16 >> 4; //shifting 16 bits to 12 bits
    float volts = (code12 / 4096.0f) * 2.56; //internal vref = 2.56V, change when using external vref
    return volts;    
}

void adc128d818::read_register(hal::byte address, std::span<hal::byte, 1> output)
{
    std::array<hal::byte, 1> address_out = {address};
    hal::write_then_read(*m_i2c, adc128d818::adc_address, address_out, output);
}

void adc128d818::read_16bit_register(hal::byte address, std::span<hal::byte, 2> output)
{
    std::array<hal::byte, 1> address_out = {address};
    hal::write_then_read(*m_i2c, adc128d818::adc_address, address_out, output);
}

void adc128d818::write_register(hal::byte address, hal::byte value)
{
    std::array<hal::byte, 2> reg_value = {address, value};
    hal::write(*m_i2c, adc128d818::adc_address, reg_value);
}
} //namespace sjsu::drivers
