#pragma once

#include "../include/swerve_module.hpp"
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/usb.hpp>
#include <libhal/zero_copy_serial.hpp>
#include <optional>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::drive {
namespace resources {
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::output_pin> status_led();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager();
hal::v5::strong_ptr<hal::input_pin> input_pin();
hal::v5::strong_ptr<hal::i2c> i2c();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::input_pin> fl_pin();
std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
can_identifier_filter();
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_left_prop(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf);
hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4>>
swerve_modules(hal::v5::strong_ptr<hal::serial> console,
               hal::v5::strong_ptr<hal::steady_clock> clock_ref,
               hal::v5::strong_ptr<hal::can_transceiver> transceiver_ref);
inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources
void initialize_platform();
void application();
}  // namespace sjsu::drive
