#pragma once

#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <optional>
namespace sjsu::drive {
struct hardware_map_t
{
  std::optional<hal::output_pin*> led;
  std::optional<hal::serial*> console;
  std::optional<hal::steady_clock*> clock;
  hal::callback<void()> reset;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
void initialize_processor();
hardware_map_t initialize_platform();
void application(hardware_map_t& p_framework);
}
