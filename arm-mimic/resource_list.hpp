#pragma once

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/adc.hpp>
#include <libhal/functional.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/usb.hpp>
#include <libhal/zero_copy_serial.hpp>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::mimic {
namespace resources {

hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::output_pin> status_led();
// define a pwm channel servo input
// separate output pin for potentiometer feedback - analog read
hal::v5::strong_ptr<hal::pwm16_channel> test_servo_pwm_channel_0();
hal::v5::strong_ptr<hal::adc> test_servo_feedback_adc_0();

// void stop();

inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources

// Application function is implemented by one of the .cpp files
void initialize_platform();
void application();
}  // namespace sjsu::mimic
