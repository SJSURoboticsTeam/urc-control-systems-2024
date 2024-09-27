#include <libhal/units.hpp>
#include <libhal/error.hpp>
#include <tla2528_adapters.hpp>

namespace sjsu::drivers {

tla2528_output_pin make_output_pin(tla2528& p_tla2528, hal::byte const p_channel, hal::output_pin::settings const& p_settings) {
    return tla2528_output_pin(p_tla2528, p_channel, p_settings);
}
tla2528_output_pin::tla2528_output_pin(tla2528& p_tla2528, hal::byte p_channel, hal::output_pin::settings const& p_settings) {
    if (p_tla2528.m_object_created & (1<<p_channel)) throw hal::resource_unavailable_try_again(this);
    p_tla2528.m_object_created = p_tla2528.m_object_created | (1 << p_channel);
    m_tla2528 = &p_tla2528;
    m_channel = p_channel;
    m_tla2528->set_channel(m_channel);
    driver_configure(p_settings);
}
tla2528_output_pin::~tla2528_output_pin(){
    m_tla2528->m_object_created = m_tla2528->m_object_created & ~(1 << m_channel);
}
void tla2528_output_pin::driver_configure(hal::output_pin::settings const& p_settings){
    if (p_settings.resistor != hal::pin_resistor::none) throw hal::operation_not_supported(this);
    m_tla2528->set_channel(m_channel);
    if (p_settings.open_drain) {
        m_tla2528->set_pin_mode(tla2528::PinMode::DigitalOutputOpenDrain);
    } else {
        m_tla2528->set_pin_mode(tla2528::PinMode::DigitalOutputPushPull);
    }
}
void tla2528_output_pin::driver_level(bool p_high) {
    m_tla2528->set_digital_out(p_high, m_channel);
}
bool tla2528_output_pin::driver_level() {
    return m_tla2528->get_digital_out(m_channel);
}


tla2528_input_pin make_input_pin(tla2528& p_tla2528, hal::byte p_channel) {
    return tla2528_input_pin(p_tla2528, p_channel);
}
tla2528_input_pin::tla2528_input_pin(tla2528& p_tla2528, hal::byte p_channel) {
    if (p_tla2528.m_object_created & (1<<p_channel)) throw hal::resource_unavailable_try_again(this);
    p_tla2528.m_object_created = p_tla2528.m_object_created | (1 << p_channel);
    m_tla2528 = &p_tla2528;
    m_channel = p_channel;
    m_tla2528->set_channel(m_channel);
    m_tla2528->set_pin_mode(tla2528::PinMode::DigitalInput);
}
tla2528_input_pin::~tla2528_input_pin(){
    m_tla2528->m_object_created = m_tla2528->m_object_created & ~(1 << m_channel);
}
//TODO: void tla2528_input_pin::driver_configure(settings const& p_settings) {}
bool tla2528_input_pin::driver_level() {
    return m_tla2528->get_digital_in(m_channel);
}


tla2528_adc make_adc(tla2528& p_tla2528, hal::byte p_channel) {
    return tla2528_adc(p_tla2528, p_channel);
}
tla2528_adc::tla2528_adc(tla2528& p_tla2528, hal::byte p_channel) {
    if (p_tla2528.m_object_created & (1<<p_channel)) throw hal::resource_unavailable_try_again(this);
    p_tla2528.m_object_created = p_tla2528.m_object_created | (1 << p_channel);
    m_tla2528 = &p_tla2528;
    m_channel = p_channel;
    m_tla2528->set_channel(m_channel);
    m_tla2528->set_pin_mode(tla2528::PinMode::AnalogInput);
}
tla2528_adc::~tla2528_adc(){
    m_tla2528->m_object_created = m_tla2528->m_object_created & ~(1 << m_channel);
}
float tla2528_adc::driver_read() {
    return m_tla2528->get_analog_in(m_channel);
}
}