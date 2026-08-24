
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

#include "h_bridge.hpp"
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
#include <libhal/can.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/rotation_sensor.hpp>
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

hal::v5::optional_ptr<hal::output_pin> output_pin_0_ptr; 
hal::v5::strong_ptr<hal::output_pin> output_pin_0()
{
  if (not output_pin_0_ptr) {
    auto output_pin_0 = gpio_a().acquire_output_pin(0); 
    output_pin_0_ptr = hal::v5::make_strong_ptr<decltype(output_pin_0)>(
      driver_allocator(), std::move(output_pin_0));
  }
  return output_pin_0_ptr; 
}

hal::v5::optional_ptr<hal::output_pin> output_pin_1_ptr; 
hal::v5::strong_ptr<hal::output_pin> output_pin_1()
{
  if (not output_pin_1_ptr) {
    auto output_pin_1 = gpio_a().acquire_output_pin(15); 
    output_pin_1_ptr = hal::v5::make_strong_ptr<decltype(output_pin_1)>(
      driver_allocator(), std::move(output_pin_1));
  }
  return output_pin_1_ptr; 
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

hal::v5::optional_ptr<hal::output_pin> pwm0_a8_ptr; 
hal::v5::strong_ptr<hal::output_pin> pwm0_a8()
{
  if (not pwm0_a8_ptr) {
    auto pwm0_a8 = gpio_a().acquire_output_pin(8);
    pwm0_a8_ptr = hal::v5::make_strong_ptr<decltype(pwm0_a8)>(
      driver_allocator(), std::move(pwm0_a8));
  }
  return pwm0_a8_ptr; 
}

hal::v5::optional_ptr<hal::output_pin> rx1_a3_ptr; 
hal::v5::strong_ptr<hal::output_pin> rx1_a3()
{
  if (not rx1_a3_ptr) {
    auto rx1_a3 = gpio_a().acquire_output_pin(3);
    rx1_a3_ptr = hal::v5::make_strong_ptr<decltype(rx1_a3)>(
      driver_allocator(), std::move(rx1_a3));
  }
  return rx1_a3_ptr; 
}

hal::v5::optional_ptr<hal::output_pin> tx1_a2_ptr; 
hal::v5::strong_ptr<hal::output_pin> tx1_a2()
{
  if (not tx1_a2_ptr) {
    auto tx1_a2 = gpio_a().acquire_output_pin(2);
    tx1_a2_ptr = hal::v5::make_strong_ptr<decltype(tx1_a2)>(
      driver_allocator(), std::move(tx1_a2));
  }
  return tx1_a2_ptr; 
}

hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_0_ptr; 
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_0()
{
  if (not pwm_channel_0_ptr) {
    auto pwm_channel_0 =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa6);
    pwm_channel_0_ptr = hal::v5::make_strong_ptr<decltype(pwm_channel_0)>(
      driver_allocator(), std::move(pwm_channel_0));
  }
  return pwm_channel_0_ptr; 
}

hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_1_ptr; 
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_1()
{
  if (not pwm_channel_1_ptr) {
    auto pwm_channel_1 =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa7);
    pwm_channel_1_ptr = hal::v5::make_strong_ptr<decltype(pwm_channel_1)>(
      driver_allocator(), std::move(pwm_channel_1));
  }
  return pwm_channel_1_ptr; 
}

hal::v5::optional_ptr<hal::pwm16_channel> pwm_channel_2_ptr; 
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel_2()
{
  if (not pwm_channel_2_ptr) {
    auto pwm_channel_2 =
      timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pb0);
    pwm_channel_2_ptr = hal::v5::make_strong_ptr<decltype(pwm_channel_2)>(
      driver_allocator(), std::move(pwm_channel_2));
  }
  return pwm_channel_2_ptr; 
}

hal::v5::optional_ptr<hal::rotation_sensor> encoder_ptr; 
hal::v5::strong_ptr<hal::rotation_sensor> encoder() 
{
  if (not encoder_ptr) {
    encoder_ptr = timer2().acquire_quadrature_encoder(
    driver_allocator(),
    {.channel_a=static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa0),
      .channel_b=static_cast<hal::stm32f1::timer_pins>(hal::stm32f1::timer2_pin::pa1) },
      1); 
      // returns ticks multiplied by 360 degrees 
      // need to divide by ticks per rotation and gear ratio to get pure degrees or linear movement 
  }
  return encoder_ptr; 
}

// all the pins for the switches at the bottom of the pcb 
hal::v5::optional_ptr<hal::input_pin> switch_g1_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g1()
{
  if (not switch_g1_ptr) {
    auto switch_g1 = gpio_a().acquire_input_pin(15); 
    switch_g1_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g1)>(
        driver_allocator(), std::move(switch_g1));
  }
  return switch_g1_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g2_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g2()
{
  if (not switch_g2_ptr) {
    auto switch_g2 = gpio_b().acquire_input_pin(3); 
    switch_g2_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g2)>(
        driver_allocator(), std::move(switch_g2));
  }
  return switch_g2_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g3_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g3()
{
  if (not switch_g3_ptr) {
    auto switch_g3 = gpio_b().acquire_input_pin(4); 
    switch_g3_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g3)>(
        driver_allocator(), std::move(switch_g3));
  }
  return switch_g3_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g4_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g4()
{
  if (not switch_g4_ptr) {
    auto switch_g4 = gpio_b().acquire_input_pin(12); 
    switch_g4_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g4)>(
        driver_allocator(), std::move(switch_g4));
  }
  return switch_g4_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g5_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g5()
{
  if (not switch_g5_ptr) {
    auto switch_g5 = gpio_b().acquire_input_pin(13); 
    switch_g5_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g5)>(
        driver_allocator(), std::move(switch_g5));
  }
  return switch_g5_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g6_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g6()
{
  if (not switch_g6_ptr) {
    auto switch_g6 = gpio_b().acquire_input_pin(14); 
    switch_g6_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g6)>(
        driver_allocator(), std::move(switch_g6));
  }
  return switch_g6_ptr;
}
hal::v5::optional_ptr<hal::input_pin> switch_g7_ptr;
hal::v5::strong_ptr<hal::input_pin> switch_g7()
{
  if (not switch_g7_ptr) {
    auto switch_g7 = gpio_b().acquire_input_pin(15); 
    switch_g7_ptr =
      hal::v5::make_strong_ptr<decltype(switch_g7)>(
        driver_allocator(), std::move(switch_g7));
  }
  return switch_g7_ptr;
}

hal::v5::optional_ptr<sjsu::drivers::h_bridge> h_bridge_ptr; 
hal::v5::strong_ptr<sjsu::drivers::h_bridge> h_bridge()
{
  if (not h_bridge_ptr) {
    // auto a_low = resources::pwm0_a8();
    auto b_low = resources::rx1_a3();
    auto c_low = resources::tx1_a2(); 
    hal::print(*console_ptr, "Acquired h-bridge low pins\n");
    // auto a_high = resources::pwm_channel_0();
    auto b_high = resources::pwm_channel_1();
    auto c_high = resources::pwm_channel_2(); 
    hal::print(*console_ptr, "Acquired h-bridge high pins\n");
    // auto h_bridge = sjsu::drivers::h_bridge({ a_high, a_low }, { b_high, b_low });
    auto h_bridge = sjsu::drivers::h_bridge(
      { .p_high=c_high, .p_low=c_low }, { .p_high=b_high, .p_low=b_low });
    h_bridge_ptr = hal::v5::make_strong_ptr<decltype(h_bridge)>(
      resources::driver_allocator(), std::move(h_bridge));
  }
  return h_bridge_ptr; 
}

hal::v5::optional_ptr<hal::stm32f1::can_peripheral_manager_v2> can_manager;
std::array<hal::v5::optional_ptr<hal::can_mask_filter>, 2> can_mask;
void initialize_can()
{
  constexpr hal::u32 baudrate = 1'000'000;
  if (not can_manager) {
    auto clock_ref = clock();
    can_manager =
      hal::v5::make_strong_ptr<hal::stm32f1::can_peripheral_manager_v2>(
        driver_allocator(),
        32,
        driver_allocator(),
        baudrate,
        *clock_ref,
        std::chrono::milliseconds(1),
        hal::stm32f1::can_pins::pb9_pb8);
    auto f = hal::acquire_can_mask_filter(driver_allocator(), can_manager);
    hal::can_mask_filter::pair p;
    p.id = 0;
    p.mask = 0;
    can_mask[0] = f[0];
    can_mask[1] = f[1];
    can_mask.at(0)->allow(p);
  }
}

hal::v5::optional_ptr<hal::can_transceiver> can_transceiver_ptr;
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  initialize_can();
  if (not can_transceiver_ptr) {
   can_transceiver_ptr = hal::acquire_can_transceiver(driver_allocator(), can_manager);
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

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  initialize_can();
  return hal::acquire_can_identifier_filter(driver_allocator(), can_manager)[0];
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