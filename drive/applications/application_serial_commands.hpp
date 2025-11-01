#pragma once

#include "../include/swerve_module.hpp"
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::drive {
namespace resources {
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::output_pin> status_led();
inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources
void initialize_platform();
void application();
}  // namespace sjsu::drive
