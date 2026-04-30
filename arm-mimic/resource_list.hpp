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
#include <libhal-actuator/rc_servo.hpp>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::mimic {
namespace resources {

hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::output_pin> status_led();
// separate output pin for potentiometer feedback - analog read
hal::v5::strong_ptr<hal::adc> a0_feedback_adc();
// define a pwm channel servo input
hal::v5::strong_ptr<hal::pwm16_channel> cipo1_pwm_channel();
hal::v5::strong_ptr<hal::actuator::rc_servo16> rc_servo();

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
