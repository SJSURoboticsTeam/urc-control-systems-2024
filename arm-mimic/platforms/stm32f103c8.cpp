// i2c
// pwms
// uart

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
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <libhal/pointers.hpp>
#include <libhal-actuator/rc_servo.hpp>


namespace sjsu::mimic::resources {
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

// extra buttons for input/output e.g. recording something
// lights, estop, button to swap modes
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

// optional pointer could be empty, initially null and then initialized to something
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

// sree promised status led
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

// Reads ADC value from A0
hal::v5::strong_ptr<hal::adc> test_servo_feedback_adc_0()
{
  static hal::atomic_spin_lock adc_lock0;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock0);
  return hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb0);
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

//pwm0 - 32 -> ch8
//pwm1 - 47 -> ch1

// Passes in PWM to CIPO1
hal::v5::strong_ptr<hal::pwm16_channel> test_servo_pwm_channel_0()
{
  auto timer_pwm_channel =
    timer3().acquire_pwm16_channel(hal::stm32f1::timer3_pin::pa6);
  timer3().acquire_pwm_group_frequency().frequency(50_Hz);
  return hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
    driver_allocator(), std::move(timer_pwm_channel));
}

// hal::v5::optional_ptr<hal::actuator::rc_servo16> rc_servo_ptr;

// hal::v5::strong_ptr<hal::actuator::rc_servo16> rc_servo()
// {
//   if (not rc_servo_ptr) {
//     hal::v5::strong_ptr<hal::pwm16_channel> pwm = test_servo_pwm_channel_0();
//     hal::actuator::rc_servo16::settings rc_servo_settings{
//       .frequency = 50,
//       .min_angle = 0,
//       .max_angle = 180,
//       .min_microseconds = 500,  // 
//       .max_microseconds = 2500, //
//     };
//     rc_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
//       driver_allocator(), pwm, rc_servo_settings);
//   }
//   return rc_servo_ptr;
// }
// hal::v5::strong_ptr<hal::actuator::rc_servo16> rc_servo()
// {
//   hal::v5::strong_ptr<hal::pwm16_channel> pwm = test_servo_pwm_channel_0();
//   hal::actuator::rc_servo16::settings rc_servo_settings{
//     .frequency = 50,
//     // Total 180 deg, change for your use case.
//     .min_angle = -90,
//     .max_angle = 90,
//     // Change to 500us and 2500us if your rc servo
//     // supports those pulse widths.
//     .min_microseconds = 750,
//     .max_microseconds = 2250,
//   };
//   static hal::actuator::rc_servo16 servo(pwm, rc_servo_settings);
//   return hal::v5::make_strong_ptr<decltype(servo)>(
//     driver_allocator(), std::move(servo));
// }

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

}  // namespace sjsu::mimic::resources
namespace sjsu::mimic {
void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(sjsu::mimic::resources::terminate_handler);
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
  //pwm0 uses pa8
  //hal::stm32f1::activate_mco_pa8(
  // hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();
}
// void resources::stop()
// stop power? tell real arm to home or stop
}  // namespace sjsu::mimic