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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
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
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>
#include <libhal/pointers.hpp>

namespace sjsu::science::resources {
using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 1024> driver_memory{};
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

hal::v5::strong_ptr<hal::adc> adc_0()
{
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
  return hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb0);
}

hal::v5::strong_ptr<hal::adc> adc_1()
{
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
  return hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb1);
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  static auto sda_output_pin = gpio_b().acquire_output_pin(7);
  static auto scl_output_pin = gpio_b().acquire_output_pin(6);
  auto clock = resources::clock();
  return hal::v5::make_strong_ptr<hal::bit_bang_i2c>(driver_allocator(),
                                                     hal::bit_bang_i2c::pins{
                                                       .sda = &sda_output_pin,
                                                       .scl = &scl_output_pin,
                                                     },
                                                     *clock);
}

hal::v5::strong_ptr<hal::input_pin> input_pin_0()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(0))>(
    driver_allocator(), gpio_a().acquire_input_pin(0));
}

hal::v5::strong_ptr<hal::input_pin> input_pin_1()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(15))>(
    driver_allocator(), gpio_a().acquire_input_pin(15));
}

hal::v5::strong_ptr<hal::input_pin> input_pin_2()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(3))>(
    driver_allocator(), gpio_b().acquire_input_pin(3));
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

auto& timer1()
{
  static hal::stm32f1::advanced_timer<st_peripheral::timer1> timer1{};
  return timer1;
}

auto& timer2()
{
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer2> timer2{};
  return timer2;
}

auto& timer3()
{
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer3> timer3{};
  return timer3;
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_0()
{
  auto timer_pwm_channel =
    timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa6);
  return hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
    driver_allocator(), std::move(timer_pwm_channel));
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_1()
{
  auto timer_pwm_channel =
    timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa7);
  return hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
    driver_allocator(), std::move(timer_pwm_channel));
}

hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency()
{
  auto timer_pwm_frequency = timer1().acquire_pwm_group_frequency();
  return hal::v5::make_strong_ptr<decltype(timer_pwm_frequency)>(
    driver_allocator(), std::move(timer_pwm_frequency));
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  throw hal::operation_not_supported(nullptr);
  // CAN is commented out in original due to potential stalling issues
  // TODO(#125): Initializing the can peripheral without it connected to a can
  // transceiver causes it to stall on occasion.
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_interrupt> can_interrupt()
{
  throw hal::operation_not_supported(nullptr);
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

}  // namespace sjsu::drivers::resources
namespace sjsu::science {
void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(sjsu::science::resources::terminate_handler);
  // Set the MCU to the maximum clock speed

  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8.0_MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
      .usb = {
        .divider = hal::stm32f1::usb_divider::divide_by_1_point_5,
      }
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

  hal::stm32f1::release_jtag_pins();
}
}  // namespace sjsu::drivers