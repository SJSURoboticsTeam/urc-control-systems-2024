#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <perseus_bldc.hpp>
namespace sjsu::drivers {

//**
// This is the class that handles TALKING to the BLDC. It is not the BLDC
// ITSELF.
//
// */
using namespace std::chrono;
using namespace hal::literals;

perseus_bldc::perseus_bldc(
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter>
    p_filter,  // this filter will allow it to see messages
               // that only this perseus cares about.
  hal::v5::strong_ptr<hal::steady_clock> clock,
  int gear_ratio,  // kind of like gear ratio
  hal::u16 servo_address)
  : m_gear_ratio(gear_ratio)
  , m_can(*transceiver, servo_address)
  , m_clock(clock)
  , m_device_id(servo_address)
{
  constexpr hal::u16 response_offest = 0x100;
  p_filter->allow(servo_address + response_offest);

  m_max_response_time = 500ms;
}

// void perseus_bldc::send_message(std::array<hal::byte, 8> const& p_payload)
// {
//   hal::can_message const payload{
//     .id = m_device_id,
//     .length = 8,
//     .payload = p_payload,
//   };

//   // Send payload
//   m_can.transceiver().send(payload);// not sure if we can actually get this information
// }

void perseus_bldc::receive_message(std::array<hal::byte, 8>& buffer)
{
  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (deadline > m_clock->uptime()) {
    if (auto const message = m_can.find(); message.has_value()) {
      buffer = message->payload;
    }
  }
  hal::safe_throw(hal::timed_out(this));
}


void perseus_bldc::set_position(hal::u16 degrees)
{
  std::array<hal::byte, 8> payload = {
    static_cast<hal::byte>(perseus_bldc::actuate::position),
    static_cast<hal::byte>(degrees >> 8), // high byte
    static_cast<hal::byte>(degrees & 0x00FF), // low byte
  };
  hal::can_message msg = {
    .id = m_device_id,
    .length = 3,
    .payload = payload
  };
  m_can.transceiver().send(msg);
}

void perseus_bldc::set_velocity(hal::i16 velocity, hal::u8 exp_bits)
{
  std::array<hal::byte, 8> payload = {
    static_cast<hal::byte>(perseus_bldc::actuate::velocity),
    static_cast<hal::byte>(exp_bits),
    static_cast<hal::byte>(velocity >> 8), // high byte
    static_cast<hal::byte>(velocity & 0x00FF), // low byte
  };
  hal::can_message msg = {
    .id = m_device_id,
    .length = 4,
    .payload = payload
  };
  m_can.transceiver().send(msg);
}


}  // namespace sjsu::drivers