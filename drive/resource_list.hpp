#pragma once

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
#include <swerve_module.hpp>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::drive {
namespace resources {
/**
 * @brief Allocator for driver memory
 *
 * The expectation is that the implementation of this allocator is a
 * std::pmr::monotonic_buffer_resource with static memory storage, meaning the
 * memory is fixed in size and memory cannot be deallocated. This is fine for
 * the demos.
 *
 * @return std::pmr::polymorphic_allocator<>
 */
std::pmr::polymorphic_allocator<> driver_allocator();
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::output_pin> status_led();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::can_identifier_filter> get_new_can_filter();
hal::v5::strong_ptr<steer_controller> front_left_steer();
hal::v5::strong_ptr<steer_controller> front_right_steer();
hal::v5::strong_ptr<steer_controller> back_right_steer();
hal::v5::strong_ptr<steer_controller> back_left_steer();
hal::v5::strong_ptr<propulsion_controller> front_left_prop();
hal::v5::strong_ptr<propulsion_controller> front_right_prop();
hal::v5::strong_ptr<propulsion_controller> back_left_prop();
hal::v5::strong_ptr<propulsion_controller> back_right_prop();
hal::v5::strong_ptr<swerve_module> front_left_swerve_module();
hal::v5::strong_ptr<swerve_module> front_right_swerve_module();
hal::v5::strong_ptr<swerve_module> back_left_swerve_module();
hal::v5::strong_ptr<swerve_module> back_right_swerve_module();
hal::v5::strong_ptr<
  std::array<hal::v5::strong_ptr<swerve_module>, module_count>>
swerve_modules();
void stop();

inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources
void initialize_platform();
void application();
}  // namespace sjsu::drive
