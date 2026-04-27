#pragma once
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/spi.hpp>

namespace sjsu::drivers {

class adc1283
{
public:
    static constexpr hal::u32 max_clock_rate = 3'200'000;
    static constexpr hal::u32 min_clock_rate = 800'000;
    static constexpr hal::byte channel_count = 8;

    // adc constructor: SPI bus, output pin, analog supply voltage
    adc1283(hal::v5::strong_ptr<hal::spi> p_spi,
            hal::v5::strong_ptr<hal::output_pin> p_cs,
            float p_avcc);
    
    // reads a single channel and returns a raw 12-bit adc code
    hal::u16 read_channel (hal::byte p_channel);

    // converts raw 12-bit ADC code to voltage
    float adc_code_to_voltage(uint16_t p_raw);

    //read a single channel and return its voltage
    float read_voltage(hal::byte p_channel);

private:
    hal::v5::strong_ptr<hal::spi> m_spi;
    hal::v5::strong_ptr<hal::output_pin> m_cs;
    float m_avcc;
};
}   // namespace sjsu::drivers