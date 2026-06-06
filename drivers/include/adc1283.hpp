#pragma once
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/spi.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

// adapter
class adc1283_adc;

/**
 * @brief driver for the adc1283 8-channel 12-bit spi adc
 *
 * The adc1283 communicates over spi and supports up to 8 single-ended analog
 * inputs. Channel selection is performed by sending a control byte. Two transfers
 * are required to retrieve data from the requested channel.
*/
class adc1283
{
public:
    static constexpr hal::u32 max_clock_rate = 3'200'000;
    static constexpr hal::u32 min_clock_rate = 800'000;
    static constexpr hal::byte channel_count = 8;

    /**
     * @param p_spi spi bus of the device
     *
     * @param p_chip_select output pin connected the adc1283's chip select
     *
     * @param p_avcc analog supply voltage 
    */
    adc1283(hal::v5::strong_ptr<hal::spi> p_spi,
            hal::v5::strong_ptr<hal::output_pin> p_chip_select,
            hal::volts p_avcc);
    
    /**
     * @brief reads a single channel and returns its 12-bit adc code
     *
     * @param p_channel channel to read
     *
     * @return raw 12-bit adc code
     * 
     * @throws hal::argument_out_of_domain if p_channel > 7
    */
    hal::u16 read_channel (hal::byte p_channel);

    /**
     * @brief reads a single channel and returns its voltage
     *
     * @param p_channel channel to read
     *
     * @return voltage in volts
     * 
     * @throws hal::argument_out_of_domain if p_channel > 7
    */
    hal::volts read_voltage(hal::byte p_channel);

    friend adc1283_adc;

private:
    /**
     * @brief converts a raw 12-bit adc code to voltage
     *
     * @param p_raw raw 12-bit adc code
     *
     * @return voltage in volts
    */
    hal::volts adc_code_to_voltage(hal::u16 p_raw);

    hal::v5::strong_ptr<hal::spi> m_spi;
    hal::v5::strong_ptr<hal::output_pin> m_chip_select;
    hal::volts m_avcc;
    hal::byte m_object_created = 0x00; // tracks
};
}   // namespace sjsu::drivers