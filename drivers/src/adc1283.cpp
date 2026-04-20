#include <libhal/error.hpp>
#include <adc1283.hpp>

namespace sjsu::drivers {

namespace{
// bit position of the channel address bits in the control register
constexpr hal::byte channel_address_shift = 3;

// mask to extract 12-bit result from 16-bit DOUT
constexpr hal::u16 result_mask = 0x0FFF;

// ADC resolution = 2^12
constexpr float adc_resolution = 4096.0f;

}   // namespace

    adc1283::adc1283(hal::v5::strong_ptr<hal::spi> p_spi,
                     hal::v5::strong_ptr<hal::output_pin> p_cs,
                     float p_avcc)
        : m_spi(p_spi)
        , m_cs(p_cs)
        , m_avcc(p_avcc)
    {
        m_spi->configure(hal::spi::settings{
            .clock_rate = max_clock_rate,
            .clock_idles_high = false,
            .data_valid_on_trailing_edge = false
        });
        m_cs->level(true);
    }

    hal::u16 adc1283::read_channel(hal::byte p_channel)
    {
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
        m_cs->level(false);
        m_spi->transfer(tx, rx, hal::spi::default_filler);
        m_spi->transfer(tx, rx, hal::spi::default_filler);
        m_cs->level(true);

        hal::u16 raw = 
            (static_cast<uint16_t>(rx[0]) << 8) | static_cast<uint16_t>(rx[1]);
        return raw & result_mask;
    }

    float adc1283::adc_code_to_voltage(uint16_t p_raw)
    {
        return (static_cast<float>(p_raw) / adc_resolution) * m_avcc;
    }

    float adc1283::read_voltage(hal::byte p_channel)
    {
        return adc_code_to_voltage(read_channel(p_channel));
    }
}