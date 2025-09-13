#include <libhal/steady_clock.hpp>
#include <perseus_bldc.hpp>

namespace sjsu::drivers {

//**
// This is the class that handles TALKING to the BLDC. It is not the BLDC
// ITSELF.
//
// */
perseus_bldc::perseus_bldc(
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::can_identifier_filter&
    p_filter,  // this filter will allow it to see messages
               // that only this pegasus cares about.
  hal::v5::strong_ptr<hal::steady_clock> clock,
  int ticks_per_rotation,  // kind of like gear ratio
  hal::u16 servo_address)
  : m_ticks_per_rotation(ticks_per_rotation)
  , m_can(*transceiver, servo_address)
  , m_clock(clock)
  , m_device_id(servo_address)
{
  hal::u16 response_offest = 0x100;
  p_filter.allow(servo_address + response_offest);
}
}  // namespace sjsu::drivers