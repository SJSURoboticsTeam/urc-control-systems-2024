
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

[[maybe_unused]] static auto& gpio_a()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_a> gpio;
  return gpio;
}
[[maybe_unused]] static auto& gpio_b()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_b> gpio;
  return gpio;
}
[[maybe_unused]] static auto& gpio_c()
{
  static hal::stm32f1::gpio<st_peripheral::gpio_c> gpio;
  return gpio;
}

static hal::v5::optional_ptr<hal::cortex_m::dwt_counter> clock_ptr;
hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (not clock_ptr) {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      driver_allocator(), cpu_frequency);
  }
  return clock_ptr;
}

static hal::v5::optional_ptr<hal::serial> console_ptr;
hal::v5::strong_ptr<hal::serial> console()
{
  if (not console_ptr) {
    console_ptr = hal::v5::make_strong_ptr<hal::stm32f1::uart>(
      driver_allocator(), hal::port<1>, hal::buffer<128>);
  }
  return console_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> led_ptr;
hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (not led_ptr) {
    auto led = gpio_c().acquire_output_pin(13);
    led_ptr = hal::v5::make_strong_ptr<decltype(led)>(driver_allocator(),
                                                      std::move(led));
  }
  return led_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> output_pin_0_ptr;
hal::v5::strong_ptr<hal::output_pin> output_pin_0()
{
  if (not output_pin_0_ptr) {
    output_pin_0_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(0))>(
        driver_allocator(), gpio_a().acquire_output_pin(0));
  }
  return output_pin_0_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> output_pin_1_ptr;
hal::v5::strong_ptr<hal::output_pin> output_pin_1()
{
  if (not output_pin_1_ptr) {
    output_pin_1_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(15))>(
        driver_allocator(), gpio_a().acquire_output_pin(15));
  }
  return output_pin_1_ptr;
}

[[maybe_unused]] static auto& timer1()
{
  static hal::stm32f1::advanced_timer<st_peripheral::timer1> timer1{};
  return timer1;
}

[[maybe_unused]] static auto& timer2()
{
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer2> timer2{};
  return timer2;
}

[[maybe_unused]] static auto& timer3()
{
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer3> timer3{};
  return timer3;
}

static hal::v5::optional_ptr<hal::output_pin> pwm0_a8_ptr;
hal::v5::strong_ptr<hal::output_pin> pwm0_a8()
{
  if (not pwm0_a8_ptr) {
    auto pin = gpio_a().acquire_output_pin(8);
    pwm0_a8_ptr = hal::v5::make_strong_ptr<decltype(pin)>(driver_allocator(),
                                                          std::move(pin));
  }
  return pwm0_a8_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> rx1_a3_ptr;
hal::v5::strong_ptr<hal::output_pin> rx1_a3()
{
  if (not rx1_a3_ptr) {
    auto pin = gpio_a().acquire_output_pin(3);
    rx1_a3_ptr = hal::v5::make_strong_ptr<decltype(pin)>(driver_allocator(),
                                                         std::move(pin));
  }
  return rx1_a3_ptr;
}
static hal::v5::optional_ptr<hal::output_pin> tx1_a2_ptr;
hal::v5::strong_ptr<hal::output_pin> tx1_a2()
{
  if (not tx1_a2_ptr) {
    auto pin = gpio_a().acquire_output_pin(2);
    tx1_a2_ptr = hal::v5::make_strong_ptr<decltype(pin)>(driver_allocator(),
                                                         std::move(pin));
  }
  return tx1_a2_ptr;
}
static hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_0_ptr;
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_0()
{
  if (not pwm_channel_0_ptr) {
    auto timer_pwm_channel =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa6);
    pwm_channel_0_ptr = hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
      driver_allocator(), std::move(timer_pwm_channel));
  }
  return pwm_channel_0_ptr;
}
static hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_1_ptr;
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_1()
{
  if (not pwm_channel_1_ptr) {
    auto timer_pwm_channel =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa7);
    pwm_channel_1_ptr = hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
      driver_allocator(), std::move(timer_pwm_channel));
  }
  return pwm_channel_1_ptr;
}
static hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_2_ptr;
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_2()
{
  if (not pwm_channel_2_ptr) {
    auto timer_pwm_channel =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pb0);
    pwm_channel_2_ptr = hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
      driver_allocator(), std::move(timer_pwm_channel));
  }
  return pwm_channel_2_ptr;
}
static hal::v5::optional_ptr<hal::rotation_sensor> encoder_ptr;
hal::v5::strong_ptr<hal::rotation_sensor> encoder()
{
  if (not encoder_ptr) {
    encoder_ptr = timer2().acquire_quadrature_encoder(
      driver_allocator(),
      { static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa0),
        static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa1) },
      1);
    // returns ticks multiplied by 360 degrees
    // need to divide by ticks per rotation and gear ratio to get pure degrees
    // or linear movement
  }
  return encoder_ptr;
}

static hal::v5::optional_ptr<sjsu::drivers::h_bridge> h_bridge_ptr;
hal::v5::strong_ptr<sjsu::drivers::h_bridge> h_bridge()
{
  // auto a_low = resources::pwm0_a8();
  auto b_low = resources::rx1_a3();
  auto c_low = resources::tx1_a2();
  hal::print(*console_ptr, "Acquired h-bridge low pins\n");
  // auto a_high = resources::pwm_channel_0();
  auto b_high = resources::pwm_channel_1();
  auto c_high = resources::pwm_channel_2();
  hal::print(*console_ptr, "Acquired h-bridge high pins\n");
  // auto h_bridge = sjsu::drivers::h_bridge({ a_high, a_low }, { b_high, b_low
  // });
  auto h_bridge = sjsu::drivers::h_bridge({ c_high, c_low }, { b_high, b_low });
  return hal::v5::make_strong_ptr<decltype(h_bridge)>(
    resources::driver_allocator(), std::move(h_bridge));
}

static hal::v5::optional_ptr<hal::stm32f1::can_peripheral_manager_v2> can_manager;
static void initialize_can()
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

static unsigned int can_filters_index = 0;
static std::array<hal::v5::optional_ptr<hal::can_identifier_filter>, 4>
  can_identifier_filters;
hal::v5::strong_ptr<hal::can_identifier_filter> get_new_can_filter()
{
  if (can_filters_index >= can_identifier_filters.size()) {
    throw hal::unknown(nullptr);  // TODO: look for better exception
  }
  if (can_filters_index % 4 == 0) {
    initialize_can();
    auto filter_batch =
      hal::acquire_can_identifier_filter(driver_allocator(), can_manager);
    for (unsigned int i = 0; i < filter_batch.size(); i++) {
      can_identifier_filters[i + can_filters_index] = filter_batch[i];
    }
  }
  auto can_id_filter = can_identifier_filters[can_filters_index];
  can_filters_index++;
  return can_id_filter;
}

static hal::v5::optional_ptr<hal::can_transceiver> can_transceiver_ptr;
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  initialize_can();
  if (not can_transceiver_ptr) {
    can_transceiver_ptr =
      hal::acquire_can_transceiver(driver_allocator(), can_manager);
  }
  return can_transceiver_ptr;
}

static hal::v5::optional_ptr<hal::can_bus_manager> can_bus_manager_ptr;
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  initialize_can();
  if (not can_bus_manager_ptr) {
    can_bus_manager_ptr =
      hal::acquire_can_bus_manager(driver_allocator(), can_manager);
  }
  return can_bus_manager_ptr;
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