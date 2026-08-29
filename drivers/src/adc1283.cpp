#include <libhal/error.hpp>
#include <adc1283.hpp>

namespace sjsu::drivers {

    adc1283::adc1283(hal::v5::strong_ptr<hal::spi> p_spi,
                     hal::v5::strong_ptr<hal::output_pin> p_chip_select,
                     hal::volts p_avcc)
        : m_spi(p_spi)
        , m_chip_select(p_chip_select)
        , m_avcc(p_avcc)
    {
        m_spi->configure(hal::spi::settings{
            .clock_rate = max_clock_rate,
            .clock_idles_high = false,
            .data_valid_on_trailing_edge = false
        });
        m_chip_select->level(true);
    }

    hal::u16 adc1283::read_channel(hal::byte p_channel)
    {
        // bit position of the channel address bits in the control register
        constexpr hal::byte channel_address_shift = 3;

        // mask to extract 12-bit result from 16-bit DOUT
        constexpr hal::u16 result_mask = 0x0FFF;

        if (p_channel >= channel_count){
            hal::safe_throw(hal::argument_out_of_domain(this));
        }
        // control byte: channel address packed into bits [5:3]
        hal::byte const control = 
            static_cast<hal::byte>(p_channel << channel_address_shift);
        
        hal::byte const tx[2] = { control, hal::spi::default_filler };
        hal::byte rx[2] = {};

        // Assert CS low to start conversion hold across both transfers
        // adc1283 always outputs channel 0 on the first conversion
        // first transfer -> primes channel selection
        // second transfer -> returns the actual result for requested channel
        m_chip_select->level(false);
        m_spi->transfer(tx, rx, hal::spi::default_filler);
        m_spi->transfer(tx, rx, hal::spi::default_filler);
        m_chip_select->level(true);

        hal::u16 raw = 
            (static_cast<hal::u16>(rx[0]) << 8) | static_cast<hal::u16>(rx[1]);
        return raw & result_mask;
    }

    hal::volts adc1283::adc_code_to_voltage(hal::u16 p_raw)
    {
        return hal::volts((static_cast<float>(p_raw) / 4096.f) * m_avcc);
    }

    hal::volts adc1283::read_voltage(hal::byte p_channel)
    {
        return adc_code_to_voltage(read_channel(p_channel));
    }
}