#pragma once

#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <optional>
namespace sjsu::drive {

// Application function must be implemented by one of the compilation units
// (.cpp) files.
void initialize_processor();
void initialize_platform();
void application();
}
