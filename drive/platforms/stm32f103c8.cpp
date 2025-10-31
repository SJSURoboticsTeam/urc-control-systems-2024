// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iterator>
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../applications/application.hpp"
#include "../include/swerve_module.hpp"
#include <limits>
#include <memory_resource>

namespace sjsu::drive::resources {
using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 4096> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

auto& gpio_a()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_a> gpio;
  return gpio;
}
auto& gpio_b()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_b> gpio;
  return gpio;
}
auto& gpio_c()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_c> gpio;
  return gpio;
}

hal::v5::optional_ptr<hal::cortex_m::dwt_counter> clock_ptr;
hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (not clock_ptr) {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      driver_allocator(), cpu_frequency);
  }
  return clock_ptr;
}

hal::v5::strong_ptr<hal::serial> console()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::uart>(
    driver_allocator(), hal::port<1>, hal::buffer<128>);
}

hal::v5::optional_ptr<hal::output_pin> led_ptr;
hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (not led_ptr) {
    auto led = gpio_c().acquire_output_pin(13);
    led_ptr = hal::v5::make_strong_ptr<decltype(led)>(driver_allocator(),
                                                      std::move(led));
  }
  return led_ptr;
}

hal::v5::optional_ptr<hal::stm32f1::can_peripheral_manager_v2> can_manager;
void initialize_can()
{
  if (not can_manager) {
    auto clock_ref = clock();
    can_manager =
      hal::v5::make_strong_ptr<hal::stm32f1::can_peripheral_manager_v2>(
        driver_allocator(),
        32,
        driver_allocator(),
        100'000,
        *clock_ref,
        std::chrono::milliseconds(1),
        hal::stm32f1::can_pins::pb9_pb8);
  }
  can_manager->baud_rate(1.0_MHz);
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  initialize_can();
  return hal::acquire_can_transceiver(driver_allocator(), can_manager);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  initialize_can();
  return hal::acquire_can_bus_manager(driver_allocator(), can_manager);
}

std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
can_identifier_filter()
{
  initialize_can();
  return hal::acquire_can_identifier_filter(driver_allocator(), can_manager);
}

hal::v5::strong_ptr<hal::input_pin> fl_pin()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(12))>(
    driver_allocator(), gpio_b().acquire_input_pin(12));  // 4
}

hal::v5::strong_ptr<hal::input_pin> fr_pin()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(13))>(
    driver_allocator(), gpio_b().acquire_input_pin(13));  // 5
}

hal::v5::strong_ptr<hal::input_pin> bl_pin()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(14))>(
    driver_allocator(), gpio_b().acquire_input_pin(14));  // 6
}

hal::v5::strong_ptr<hal::input_pin> br_pin()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(15))>(
    driver_allocator(), gpio_b().acquire_input_pin(15));  // 7
}

hal::v5::strong_ptr<hal::output_pin> output_pin_0()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(0))>(
    driver_allocator(), gpio_a().acquire_output_pin(0));
}

hal::v5::strong_ptr<hal::output_pin> output_pin_1()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(15))>(
    driver_allocator(), gpio_a().acquire_output_pin(15));
}

hal::v5::strong_ptr<hal::output_pin> output_pin_2()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(3))>(
    driver_allocator(), gpio_b().acquire_output_pin(3));
}

hal::v5::strong_ptr<hal::output_pin> output_pin_3()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(4))>(
    driver_allocator(), gpio_b().acquire_output_pin(4));
}

hal::v5::strong_ptr<hal::output_pin> output_pin_4()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(12))>(
    driver_allocator(), gpio_b().acquire_output_pin(12));
}

std::array<hal::v5::strong_ptr<hal::output_pin>, 5> std::output_pins()
{
  return
  {
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(0))>(
    driver_allocator(), gpio_a().acquire_output_pin(0));
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(15))>(
    driver_allocator(), gpio_a().acquire_output_pin(15));
    hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(3))>(
    driver_allocator(), gpio_b().acquire_output_pin(3));
    hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(4))>(
    driver_allocator(), gpio_b().acquire_output_pin(4));
    hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(12))>(
    driver_allocator(), gpio_b().acquire_output_pin(12));
  }
}
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_left_steer(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x14C);
    ;
  } catch (hal::exception e) {
    print<64>(
      *console, "Front left steer failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_left_prop(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x141);
    ;
  } catch (hal::exception e) {
    print<64>(
      *console, "Front left prop failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_right_steer(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x142);
    ;
  } catch (hal::exception e) {
    print<64>(
      *console, "Front right steer failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_right_prop(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x14D);
    ;
  } catch (hal::exception e) {
    print<64>(
      *console, "Front right prop failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_left_steer(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x144);
  } catch (hal::exception e) {
    print<64>(
      *console, "back left steer failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_left_prop(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x141);
  } catch (hal::exception e) {
    print<64>(
      *console, "back left prop failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_right_steer(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x14F);
  } catch (hal::exception e) {
    print<64>(
      *console, "back right steer failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_right_prop(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf)
{
  try {
    return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
      driver_allocator(), *transceiver, *idf, *clock, 36.0f, 0x153);
  } catch (hal::exception e) {
    print<64>(
      *console, "back right prop failed, error code: \n", e.error_code());
    throw e;
  }
}

hal::v5::strong_ptr<swerve_module> front_left_swerve_module(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf0,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf1)
{
  static swerve_module_settings front_left_settings{ .position =
                                                       vector2d{ 0.3f, 0.3f },
                                                     .steer_offset = 0,
                                                     .max_speed = 10,
                                                     .acceleration = 4.0,
                                                     .turn_speed = 360.0,
                                                     .min_angle = -180.0,
                                                     .max_angle = 180.0,
                                                     .position_tolerance = 5.0,
                                                     .velocity_tolerance = 0.5,
                                                     .tolerance_timeout = 0.5,
                                                     .reversed = true };
  return hal::v5::make_strong_ptr<swerve_module>(
    driver_allocator(),
    front_left_steer(console, clock, transceiver, idf0),
    front_left_prop(console, clock, transceiver, idf1),
    fl_pin(),
    clock,
    console,
    front_left_settings);
}

hal::v5::strong_ptr<swerve_module> front_right_swerve_module(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf0,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf1)
{
  swerve_module_settings front_right_settings{ .position =
                                                 vector2d{ 0.3f, 0.3f },
                                               .steer_offset = 0,
                                               .max_speed = 10,
                                               .acceleration = 4.0,
                                               .turn_speed = 360.0,
                                               .min_angle = -180.0,
                                               .max_angle = 180.0,
                                               .position_tolerance = 5.0,
                                               .velocity_tolerance = 0.5,
                                               .tolerance_timeout = 0.5,
                                               .reversed = false };
  return hal::v5::make_strong_ptr<swerve_module>(
    driver_allocator(),
    front_right_steer(console, clock, transceiver, idf0),
    front_right_prop(console, clock, transceiver, idf1),
    fr_pin(),
    clock,
    console,
    front_right_settings);
}

hal::v5::strong_ptr<swerve_module> back_left_swerve_module(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf0,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf1)
{
  static swerve_module_settings back_left_settings{ .position =
                                                      vector2d{ 0.3f, 0.3f },
                                                    .steer_offset = 0,
                                                    .max_speed = 10,
                                                    .acceleration = 4.0,
                                                    .turn_speed = 360.0,
                                                    .min_angle = -180.0,
                                                    .max_angle = 180.0,
                                                    .position_tolerance = 5.0,
                                                    .velocity_tolerance = 0.5,
                                                    .tolerance_timeout = 0.5,
                                                    .reversed = true };
  return hal::v5::make_strong_ptr<swerve_module>(
    driver_allocator(),
    back_left_steer(console, clock, transceiver, idf0),
    back_left_prop(console, clock, transceiver, idf1),
    bl_pin(),
    clock,
    console,
    back_left_settings);
}

hal::v5::strong_ptr<swerve_module> back_right_swerve_module(
  hal::v5::strong_ptr<hal::serial> console,
  hal::v5::strong_ptr<hal::steady_clock> clock,
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf0,
  hal::v5::strong_ptr<hal::can_identifier_filter> idf1)
{
  hal::print(*console, "Making back right swerve module\n");
  static swerve_module_settings back_right_settings{ .position =
                                                       vector2d{ 0.3f, 0.3f },
                                                     .steer_offset = 0,
                                                     .max_speed = 10,
                                                     .acceleration = 4.0,
                                                     .turn_speed = 360.0,
                                                     .min_angle = -180.0,
                                                     .max_angle = 180.0,
                                                     .position_tolerance = 5.0,
                                                     .velocity_tolerance = 0.5,
                                                     .tolerance_timeout = 0.5,
                                                     .reversed = false };
  return hal::v5::make_strong_ptr<swerve_module>(
    driver_allocator(),
    back_right_steer(console, clock, transceiver, idf0),
    back_right_prop(console, clock, transceiver, idf1),
    br_pin(),
    clock,
    console,
    back_right_settings);
}

hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4>>
swerve_modules(hal::v5::strong_ptr<hal::serial> console,
               hal::v5::strong_ptr<hal::steady_clock> clock_ref,
               hal::v5::strong_ptr<hal::can_transceiver> transceiver_ref)
{

  static auto idf_set_0 = can_identifier_filter();
  static auto idf_set_1 = can_identifier_filter();
  hal::print(*console, "identifier filter created\n");
  static auto modules = std::array<hal::v5::strong_ptr<swerve_module>, 4>{
    front_left_swerve_module(
      console, clock_ref, transceiver_ref, idf_set_0[0], idf_set_0[1]),
    front_right_swerve_module(
      console, clock_ref, transceiver_ref, idf_set_0[2], idf_set_0[3]),
    back_left_swerve_module(
      console, clock_ref, transceiver_ref, idf_set_1[0], idf_set_1[1]),
    back_right_swerve_module(
      console, clock_ref, transceiver_ref, idf_set_1[2], idf_set_1[3])
  };
  return hal::v5::make_strong_ptr<
    std::array<hal::v5::strong_ptr<swerve_module>, 4>>(driver_allocator(),
                                                       std::move(modules));
}

[[noreturn]] void terminate_handler() noexcept
{
  if (not led_ptr && not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern
  auto status_led = resources::status_led();
  auto clock = resources::clock();

  while (true) {
    using namespace std::chrono_literals;
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 100ms);
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 1000ms);
  }
}

}  // namespace sjsu::drive::resources

namespace sjsu::drive {
void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(resources::terminate_handler);
  // Set the MCU to the maximum clock speed

  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8.0_MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
    },
    .system_clock = hal::stm32f1::system_clock_select::pll,
    .ahb = {
      .divider = hal::stm32f1::ahb_divider::divide_by_1,
      .apb1 = {
        .divider = hal::stm32f1::apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = hal::stm32f1::apb_divider::divide_by_1,
        .adc = {
          .divider = hal::stm32f1::adc_divider::divide_by_6,
        }
      },
    },
  });
  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();
}
}  // namespace sjsu::drive
