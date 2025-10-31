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

#include <array>
#include <cstdint>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
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
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <limits>
#include <memory_resource>
#include <resource_list.hpp>
#include <swerve_module.hpp>

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

hal::v5::optional_ptr<hal::serial> console_ptr;
hal::v5::strong_ptr<hal::serial> console()
{
  if (not console_ptr) {
    console_ptr = hal::v5::make_strong_ptr<hal::stm32f1::uart>(
      driver_allocator(), hal::port<1>, hal::buffer<128>);
  }
  return console_ptr;
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
    can_manager->baud_rate(1.0_MHz);
  }
}

std::array<std::array<hal::v5::optional_ptr<hal::can_identifier_filter>, 4>, 2>
  can_identifier_filters;
hal::v5::strong_ptr<hal::can_identifier_filter> get_new_can_filter()
{
  static unsigned int can_filters_array_index = 0;
  if (can_filters_array_index >=
      can_identifier_filters.size() * can_identifier_filters[0].size()) {
    // TODO replace with a better exception
    throw hal::unknown(nullptr);
  }
  if (can_filters_array_index % 4) {
    initialize_can();
    auto filter_batch =
      hal::acquire_can_identifier_filter(driver_allocator(), can_manager);
    for (unsigned int i = 0; i < filter_batch.size(); i++) {
      can_identifier_filters[can_filters_array_index / 4][i] = filter_batch[i];
    }
  }
  auto can_id_filter = can_identifier_filters[can_filters_array_index / 4]
                                             [can_filters_array_index % 4];
  can_filters_array_index++;
  return can_id_filter;
}

hal::v5::optional_ptr<hal::can_transceiver> can_transceiver_ptr;
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  initialize_can();
  if (not can_transceiver_ptr) {
    can_transceiver_ptr =
      hal::acquire_can_transceiver(driver_allocator(), can_manager);
  }
  return can_transceiver_ptr;
}

hal::v5::optional_ptr<hal::can_bus_manager> can_bus_manager_ptr;
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  initialize_can();
  if (not can_bus_manager_ptr) {
    can_bus_manager_ptr =
      hal::acquire_can_bus_manager(driver_allocator(), can_manager);
  }
  return can_bus_manager_ptr;
}

hal::v5::optional_ptr<hal::input_pin> front_left_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> front_left_limit_switch()
{
  if (not front_left_limit_switch_ptr) {
    auto front_left_limit_switch = gpio_b().acquire_input_pin(12);  // 4
    front_left_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(front_left_limit_switch)>(
        driver_allocator(), std::move(front_left_limit_switch));
  }
  return front_left_limit_switch_ptr;
}

hal::v5::optional_ptr<hal::input_pin> front_right_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> front_right_limit_switch()
{
  if (not front_right_limit_switch_ptr) {
    auto front_right_limit_switch = gpio_b().acquire_input_pin(13);  // 5
    front_right_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(front_right_limit_switch)>(
        driver_allocator(), std::move(front_right_limit_switch));
  }
  return front_right_limit_switch_ptr;
}

hal::v5::optional_ptr<hal::input_pin> back_left_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> back_left_limit_switch()
{
  if (not back_left_limit_switch_ptr) {
    auto back_left_limit_switch = gpio_b().acquire_input_pin(14);  // 6
    back_left_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(back_left_limit_switch)>(
        driver_allocator(), std::move(back_left_limit_switch));
  }
  return back_left_limit_switch_ptr;
}

hal::v5::optional_ptr<hal::input_pin> back_right_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> back_right_limit_switch()
{
  if (not back_right_limit_switch_ptr) {
    auto back_right_limit_switch = gpio_b().acquire_input_pin(15);  // 7
    back_right_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(back_right_limit_switch)>(
        driver_allocator(), std::move(back_right_limit_switch));
  }
  return back_right_limit_switch_ptr;
}

hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> make_rmd(uint16_t p_address)
{
  auto console_ref = resources::console();
  auto clock_ref = resources::clock();
  auto transceiver = resources::can_transceiver();
  auto idf = get_new_can_filter();
  return hal::v5::make_strong_ptr<hal::actuator::rmd_mc_x_v2>(
    driver_allocator(), *transceiver, *idf, *clock_ref, 36.0f, p_address);
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> front_left_steer_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_left_steer()
{
  if (not front_left_steer_ptr) {
    try {
      front_left_steer_ptr = make_rmd(0x14C);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(*console_ref, "Front left steer failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return front_left_steer_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> front_left_prop_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_left_prop()
{
  if (not front_left_prop_ptr) {
    try {
      front_left_prop_ptr = make_rmd(0x141);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(
        *console_ref, "Front left prop failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return front_left_prop_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> front_right_steer_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_right_steer()
{
  if (not front_right_steer_ptr) {
    try {
      front_right_steer_ptr = make_rmd(0x142);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(*console_ref, "Front right steer failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return front_right_steer_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> front_right_prop_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> front_right_prop()
{
  if (not front_right_prop_ptr) {
    try {
      front_right_prop_ptr = make_rmd(0x14D);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(*console_ref, "Front right prop failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return front_right_prop_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> back_left_steer_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_left_steer()
{
  if (not back_left_steer_ptr) {
    try {
      back_left_steer_ptr = make_rmd(0x144);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(
        *console_ref, "back left steer failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return back_left_steer_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> back_left_prop_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_left_prop()
{
  if (not back_left_prop_ptr) {
    try {
      back_left_prop_ptr = make_rmd(0x141);
    } catch (hal::exception e) {
      auto console_ref = resources::console();
      print<64>(
        *console_ref, "back left prop failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return back_left_prop_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> back_right_steer_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_right_steer()
{
  if (not back_right_steer_ptr) {
    try {
      back_right_steer_ptr = make_rmd(0x14F);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(*console_ref, "back right steer failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return back_right_steer_ptr;
}

hal::v5::optional_ptr<hal::actuator::rmd_mc_x_v2> back_right_prop_ptr;
hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> back_right_prop()
{
  if (not back_right_prop_ptr) {
    try {
      back_right_prop_ptr = make_rmd(0x153);
    } catch (hal::exception e) {
      auto console_ref = console();
      print<64>(
        *console_ref, "back right prop failed, error code: \n", e.error_code());
      throw e;
    }
  }
  return back_right_prop_ptr;
}

constexpr swerve_module_settings base_module_settings{
  .max_speed = 10,
  .acceleration = 4.0,
  .turn_speed = 360.0,
  .min_angle = -135.0,
  .max_angle = 135.0,
  .position_tolerance = 5.0,
  .velocity_tolerance = 0.5,
  .tolerance_timeout = 0.5,
};
hal::v5::optional_ptr<swerve_module> front_left_swerve_module_ptr;
hal::v5::strong_ptr<swerve_module> front_left_swerve_module()
{
  if (not front_left_swerve_module_ptr) {
    constexpr swerve_module_settings front_left_settings{
      .position = vector2d(0.0, 0.0),
      .max_speed = base_module_settings.max_speed,
      .acceleration = base_module_settings.acceleration,
      .turn_speed = base_module_settings.turn_speed,
      .min_angle = base_module_settings.min_angle,
      .max_angle = base_module_settings.max_angle,
      .limit_switch_position = 135.0,
      .position_tolerance = base_module_settings.position_tolerance,
      .velocity_tolerance = base_module_settings.velocity_tolerance,
      .tolerance_timeout = base_module_settings.tolerance_timeout,
      .home_clockwise = false
    };
    front_left_swerve_module_ptr =
      hal::v5::make_strong_ptr<swerve_module>(driver_allocator(),
                                              front_left_steer(),
                                              front_left_prop(),
                                              front_left_limit_switch(),
                                              clock(),
                                              front_left_settings);
  }
  return front_left_swerve_module_ptr;
}
hal::v5::optional_ptr<swerve_module> front_right_swerve_module_ptr;
hal::v5::strong_ptr<swerve_module> front_right_swerve_module()
{
  if (not front_right_swerve_module_ptr) {
    constexpr swerve_module_settings front_right_settings{
      .position = vector2d(0.0, 0.0),
      .max_speed = base_module_settings.max_speed,
      .acceleration = base_module_settings.acceleration,
      .turn_speed = base_module_settings.turn_speed,
      .min_angle = base_module_settings.min_angle,
      .max_angle = base_module_settings.max_angle,
      .limit_switch_position = -135.0,
      .position_tolerance = base_module_settings.position_tolerance,
      .velocity_tolerance = base_module_settings.velocity_tolerance,
      .tolerance_timeout = base_module_settings.tolerance_timeout,
      .home_clockwise = true
    };
    front_right_swerve_module_ptr =
      hal::v5::make_strong_ptr<swerve_module>(driver_allocator(),
                                              front_right_steer(),
                                              front_right_prop(),
                                              front_right_limit_switch(),
                                              clock(),
                                              front_right_settings);
  }
  return front_right_swerve_module_ptr;
}

hal::v5::optional_ptr<swerve_module> back_left_swerve_module_ptr;
hal::v5::strong_ptr<swerve_module> back_left_swerve_module()
{
  if (not back_left_swerve_module_ptr) {
    constexpr swerve_module_settings back_left_settings{
      .position = vector2d(0.0, 0.0),
      .max_speed = base_module_settings.max_speed,
      .acceleration = base_module_settings.acceleration,
      .turn_speed = base_module_settings.turn_speed,
      .min_angle = base_module_settings.min_angle,
      .max_angle = base_module_settings.max_angle,
      .limit_switch_position = 135.0,
      .position_tolerance = base_module_settings.position_tolerance,
      .velocity_tolerance = base_module_settings.velocity_tolerance,
      .tolerance_timeout = base_module_settings.tolerance_timeout,
      .home_clockwise = false
    };
    back_left_swerve_module_ptr =
      hal::v5::make_strong_ptr<swerve_module>(driver_allocator(),
                                              back_left_steer(),
                                              back_left_prop(),
                                              back_left_limit_switch(),
                                              clock(),
                                              back_left_settings);
  }
  return back_left_swerve_module_ptr;
}

hal::v5::optional_ptr<swerve_module> back_right_swerve_module_ptr;
hal::v5::strong_ptr<swerve_module> back_right_swerve_module()
{
  if (not back_right_swerve_module_ptr) {
    constexpr swerve_module_settings back_right_settings{
      .position = vector2d(0.0, 0.0),
      .max_speed = base_module_settings.max_speed,
      .acceleration = base_module_settings.acceleration,
      .turn_speed = base_module_settings.turn_speed,
      .min_angle = base_module_settings.min_angle,
      .max_angle = base_module_settings.max_angle,
      .limit_switch_position = -135.0,
      .position_tolerance = base_module_settings.position_tolerance,
      .velocity_tolerance = base_module_settings.velocity_tolerance,
      .tolerance_timeout = base_module_settings.tolerance_timeout,
      .home_clockwise = true
    };
    back_right_swerve_module_ptr =
      hal::v5::make_strong_ptr<swerve_module>(driver_allocator(),
                                              back_right_steer(),
                                              back_right_prop(),
                                              back_right_limit_switch(),
                                              clock(),
                                              back_right_settings);
  }
  return back_right_swerve_module_ptr;
}

hal::v5::optional_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4>>
  swerve_modules_ptr;
hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4>>
swerve_modules()
{
  if (not swerve_modules_ptr) {
    auto modules = std::array<hal::v5::strong_ptr<swerve_module>, 4>{
      front_left_swerve_module(),
      front_right_swerve_module(),
      back_left_swerve_module(),
      back_right_swerve_module()
    };
    swerve_modules_ptr = hal::v5::make_strong_ptr<
      std::array<hal::v5::strong_ptr<swerve_module>, 4>>(driver_allocator(),
                                                         std::move(modules));
  }
  return swerve_modules_ptr;
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
  auto clock_ref = resources::clock();

  while (true) {
    using namespace std::chrono_literals;
    status_led->level(false);
    hal::delay(*clock_ref, 100ms);
    status_led->level(true);
    hal::delay(*clock_ref, 100ms);
    status_led->level(false);
    hal::delay(*clock_ref, 100ms);
    status_led->level(true);
    hal::delay(*clock_ref, 1000ms);
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
