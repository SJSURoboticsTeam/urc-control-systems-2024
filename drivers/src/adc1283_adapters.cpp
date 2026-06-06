#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <adc1283_adapters.hpp>

namespace sjsu::drivers {

adc1283_adc::adc1283_adc(adc1283& p_adc1283, hal::byte p_channel)
    : m_adc1283(&p_adc1283)
    , m_channel(p_channel)
{
    if (hal::bit_extract(hal::bit_mask::from(m_channel),
                         m_adc1283->m_object_created)) {
        hal::safe_throw(hal::resource_unavailable_try_again(this));
    }
    hal::bit_modify(m_adc1283->m_object_created)
        .set(hal::bit_mask::from(m_channel));
}

adc1283_adc::~adc1283_adc()
{
    hal::bit_modify(m_adc1283->m_object_created)
        .clear(hal::bit_mask::from(m_channel));
}

float adc1283_adc::driver_read()
{
    return static_cast<float>(m_adc1283->read_channel(m_channel)) / 4096.0f;
}

adc1283_adc make_adc(adc1283& p_adc1283, hal::byte p_channel)
{
    return adc1283_adc(p_adc1283, p_channel);
}

} // namespace sjsu::drivers