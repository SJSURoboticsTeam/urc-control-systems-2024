
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
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>

#include <resource_list.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <resource_list.hpp>

namespace sjsu::perseus::resources {
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

hal::v5::strong_ptr<hal::output_pin> pwm0_a8()
{
  auto pin = gpio_a().acquire_output_pin(8);
  return hal::v5::make_strong_ptr<decltype(pin)>(driver_allocator(),
                                                 std::move(pin));
}

hal::v5::strong_ptr<hal::output_pin> rx1_a3()
{
  auto pin = gpio_a().acquire_output_pin(3);
  return hal::v5::make_strong_ptr<decltype(pin)>(driver_allocator(),
                                                 std::move(pin));
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
hal::v5::strong_ptr<hal::rotation_sensor> encoder() 
{
  return timer2().acquire_quadrature_encoder(
    driver_allocator(),
    { static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa0),
      static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa1) },
      720); 
      //this is to just get the plain number of ticks, divided by two bc it reports both A and B channel
}
hal::v5::strong_ptr<sjsu::drivers::h_bridge> h_bridge()
{
  auto a_low = resources::pwm0_a8();
  auto b_low = resources::rx1_a3();
  hal::print(*console_ptr, "Acquired h-bridge low pins\n");
  auto a_high = resources::pwm_channel_0();
  auto b_high = resources::pwm_channel_1();
  hal::print(*console_ptr, "Acquired h-bridge high pins\n");
  auto h_bridge = sjsu::drivers::h_bridge({ a_high, a_low }, { b_high, b_low });
  return hal::v5::make_strong_ptr<decltype(h_bridge)>(
    resources::driver_allocator(), std::move(h_bridge));
}


// add one for quadrature encoder

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

}  // namespace sjsu::perseus::resources
namespace sjsu::perseus {
void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(sjsu::perseus::resources::terminate_handler);
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
}  // namespace sjsu::perseus