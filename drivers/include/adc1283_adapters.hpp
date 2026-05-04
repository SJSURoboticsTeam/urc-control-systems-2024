#pragma once
#include <libhal/adc.hpp>
#include <adc1283.hpp>

namespace sjsu::drivers {

class adc1283_adc : public hal::adc
{
public:
    ~adc1283_adc();
    friend adc1283_adc make_adc(adc1283& p_adc1283, hal::byte p_channel);

private:
    adc1283_adc(adc1283& p_adc1283, hal::byte p_channel);
    float driver_read();

    adc1283* m_adc1283 = nullptr;
    hal::byte m_channel;
};

/**
 * @brief create a hal::adc driver using the adc1283 driver
 *
 * @param p_adc1283 reference to the adc1283 driver
 *
 * @param p_channel channel to read
 *
 * @throws hal::argument_out_of_domain if p_channel > 7
*/
adc1283_adc make_adc(adc1283& p_adc1283, hal::byte p_channel);

} // namespace sjsu::drivers