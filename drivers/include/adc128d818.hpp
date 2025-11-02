#pragma once

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>

namespace sjsu::drivers{
class adc128d818
{
public:
    enum register_addresses : hal::byte
    {
        config = 0x00,
        interrupt_status = 0x01,
        interrupt_mask = 0x03,
        conv_rate = 0x07,
        ch_enable = 0x08,
        one_shot = 0x09,
        deep_shutdown = 0x0A,
        adv_config = 0x0B,
        busy_status = 0x0C,

        //channel reading registers MODE 0
        in0 = 0x20, //vADC
        in1 = 0x21, //ADC_1
        in2 = 0x22, //x
        in3 = 0x23, //x
        in4 = 0x24, //ADC_2
        in5 = 0x25, //ADC_3
        in6 = 0x26, //x
        in7 = 0x27  //ADC_4
    };

    enum bit_masks : hal::byte
    {
        busy = 0x20,
        mode_0 = 0x00, //for advanced config register, mode0 and using internal vref (2.56)
        cont_conv = 0x01, //for conversion rate register
        enable_ch = 0x4C, //enabling channels: IN0, IN1, IN4, IN5, IN7
        start = 0x01 //for config register, use to start channel readings
    };

    hal::byte adc_address = 0x1D; //assuming low low, need to check this
    float vref_ext = 3.3f;  //check this once we switch to external vref -also switch from mode0

    adc128d818(hal::v5::strong_ptr<hal::i2c> p_i2c,
               hal::v5::strong_ptr<hal::steady_clock> p_clock,
               hal::v5::strong_ptr<hal::serial> p_terminal);

    void startup_sequence();

    float read_voltage();

    void read_register(hal::byte address, std::span<hal::byte, 1> output);
    void read_16bit_register(hal::byte address, std::span<hal::byte, 2> output);
    void write_register(hal::byte address, hal::byte value);


    hal::v5::strong_ptr<hal::i2c>  m_i2c;
    hal::v5::strong_ptr<hal::steady_clock> m_clock;
    hal::v5::strong_ptr<hal::serial> m_terminal;
};
} // namespace sjsu::drivers