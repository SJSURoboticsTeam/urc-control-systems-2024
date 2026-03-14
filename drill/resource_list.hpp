#pragma once

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/usb.hpp>
#include <libhal/zero_copy_serial.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>


// Application function must be implemented by one of the compilation units
// (.cpp) files.
namespace sjsu::drill {
namespace resources {

  hal::v5::strong_ptr<hal::steady_clock> clock();
  hal::v5::strong_ptr<hal::serial> console();
  hal::v5::strong_ptr<hal::output_pin> status_led();
  hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
  hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager();
  hal::v5::strong_ptr<hal::v5::can_identifier_filter> can_filter();
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> drill();
  hal::v5::strong_ptr<hal::i2c> i2c();
  hal::v5::strong_ptr<hal::output_pin> output_pin_5();
  hal::v5::strong_ptr<hal::output_pin> output_pin_0();
  hal::v5::strong_ptr<hal::output_pin> output_pin_7();
  hal::v5::strong_ptr<hal::output_pin> output_pin_6();
  hal::v5::strong_ptr<hal::output_pin> output_pin_4();
   


  inline void reset()
  {
    hal::cortex_m::reset();
  }
}  // namespace resources
void initialize_platform();
void application();
}  // namespace sjsu::drill
