#pragma once

#include "steering_module.hpp"
#include "steering_module.hpp"

#include <libhal/units.hpp>
#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal-util/serial.hpp>
#include <span>
#include <cmath>
#include <cstddef>

namespace sjsu::drive {
void send_custom_message(hal::u32 p_id,
                         hal::can_transceiver& p_can,
                         hal::u8 p_length,
                         std::array<hal::byte, 8>&& p_payload);

void home(std::span<steering_module> legs,
          std::span<start_wheel_setting> setting_span,
          hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal);


const hal::u8 reset_command = 0x80;
const hal::u8 encoder_zero_command = 0x64;

}  // namespace sjsu::drive