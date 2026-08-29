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

#include <libhal-actuator/rc_servo.hpp>
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
#include <libhal-expander/pca9685.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <libhal/pointers.hpp>
#include <resource_list.hpp>

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

static hal::v5::optional_ptr<hal::expander::pca9685> pca_ptr;
hal::v5::strong_ptr<hal::expander::pca9685> pca()
{
  if (not pca_ptr) {
    auto i2c = resources::i2c();
    pca_ptr = hal::v5::make_strong_ptr<hal::expander::pca9685>(
      driver_allocator(), *i2c, 0b100'0000);
  }
  return pca_ptr;
}

hal::v5::optional_ptr<hal::adc> adc_0_ptr;
hal::v5::strong_ptr<hal::adc> adc_0()
{
  if (not adc_0_ptr) {
    static hal::atomic_spin_lock adc_lock;
    static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
    adc_0_ptr =
      hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb0);
  }
  return adc_0_ptr;
}

hal::v5::optional_ptr<hal::adc> adc_1_ptr;
hal::v5::strong_ptr<hal::adc> adc_1()
{
  if (not adc_1_ptr) {
    static hal::atomic_spin_lock adc_lock;
    static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
    adc_1_ptr =
      hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb1);
  }
  return adc_1_ptr;
}

static hal::v5::optional_ptr<hal::i2c> i2c_ptr;
hal::v5::strong_ptr<hal::i2c> i2c()
{
  if (not i2c_ptr) {
    static auto sda_output_pin = gpio_b().acquire_output_pin(7);
    static auto scl_output_pin = gpio_b().acquire_output_pin(6);
    auto clock = resources::clock();
    i2c_ptr =
      hal::v5::make_strong_ptr<hal::bit_bang_i2c>(driver_allocator(),
                                                  hal::bit_bang_i2c::pins{
                                                    .sda = &sda_output_pin,
                                                    .scl = &scl_output_pin,
                                                  },
                                                  *clock);
  }
  return i2c_ptr;
}

static hal::v5::optional_ptr<hal::input_pin> input_pin_0_ptr;
hal::v5::strong_ptr<hal::input_pin> input_pin_0()
{
  if (not input_pin_0_ptr) {
    input_pin_0_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(0))>(
        driver_allocator(), gpio_a().acquire_input_pin(0));
  }
  return input_pin_0_ptr;
}

static hal::v5::optional_ptr<hal::input_pin> input_pin_1_ptr;
hal::v5::strong_ptr<hal::input_pin> input_pin_1()
{
  if (not input_pin_1_ptr) {
    input_pin_1_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(15))>(
        driver_allocator(), gpio_a().acquire_input_pin(15));
  }
  return input_pin_1_ptr;
}

static hal::v5::optional_ptr<hal::input_pin> input_pin_2_ptr;
hal::v5::strong_ptr<hal::input_pin> input_pin_2()
{
  if (not input_pin_2_ptr) {
    input_pin_2_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(3))>(
        driver_allocator(), gpio_b().acquire_input_pin(3));
  }
  return input_pin_2_ptr;
}

static hal::v5::optional_ptr<hal::input_pin> top_door_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> top_door_limit_switch()
{
  if (not top_door_limit_switch_ptr) {
    auto top_door_limit_switch = gpio_b().acquire_input_pin(
      14);  // 6  // GPIO AND PIN TBD WHEN SCIENCE BOARD SCHEMATIC GIVEN
    top_door_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(top_door_limit_switch)>(
        driver_allocator(), std::move(top_door_limit_switch));
  }
  return top_door_limit_switch_ptr;
}

static hal::v5::optional_ptr<hal::input_pin> bottom_door_limit_switch_ptr;
hal::v5::strong_ptr<hal::input_pin> bottom_door_limit_switch()
{
  if (not bottom_door_limit_switch_ptr) {
    auto bottom_door_limit_switch = gpio_b().acquire_input_pin(
      15);  // 7  // GPIO AND PIN TBD WHEN SCIENCE BOARD SCHEMATIC GIVEN
    bottom_door_limit_switch_ptr =
      hal::v5::make_strong_ptr<decltype(bottom_door_limit_switch)>(
        driver_allocator(), std::move(bottom_door_limit_switch));
  }
  return bottom_door_limit_switch_ptr;
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

static hal::v5::optional_ptr<hal::output_pin> kalling_reagent_pump_ptr;
hal::v5::strong_ptr<hal::output_pin> kalling_reagent_pump()
{
  if (not kalling_reagent_pump_ptr) {
    kalling_reagent_pump_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(15))>(
        driver_allocator(), gpio_a().acquire_output_pin(15));
  }
  return kalling_reagent_pump_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> biuret_reagent_pump_ptr;
hal::v5::strong_ptr<hal::output_pin> biuret_reagent_pump()
{
  if (not biuret_reagent_pump_ptr) {
    biuret_reagent_pump_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(3))>(
        driver_allocator(), gpio_b().acquire_output_pin(3));
  }
  return biuret_reagent_pump_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> benedict_reagent_pump_ptr;
hal::v5::strong_ptr<hal::output_pin> benedict_reagent_pump()
{
  if (not benedict_reagent_pump_ptr) {
    benedict_reagent_pump_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(4))>(
        driver_allocator(), gpio_b().acquire_output_pin(4));
  }
  return benedict_reagent_pump_ptr;
}

static hal::v5::optional_ptr<hal::output_pin> deionized_water_pump_ptr;
hal::v5::strong_ptr<hal::output_pin> deionized_water_pump()
{
  if (not deionized_water_pump_ptr) {
    deionized_water_pump_ptr =
      hal::v5::make_strong_ptr<decltype(gpio_b().acquire_output_pin(12))>(
        driver_allocator(), gpio_b().acquire_output_pin(12));
  }
  return deionized_water_pump_ptr;
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

static hal::v5::optional_ptr<hal::actuator::rc_servo> mixer_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> mixer_servo()
{
  if (not mixer_servo_ptr) {
    static auto servo_pca_ptr = pca();
    static auto mixer_pwm0 = servo_pca_ptr->get_pwm_channel<0>();
    constexpr hal::actuator::rc_servo::settings mixer_servo_settings{
      // WHAT IS MIXER SPECS
      .frequency = 50,         .min_angle = 0,           .max_angle = 190,
      .min_microseconds = 600, .max_microseconds = 2400,
    };
    mixer_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), mixer_pwm0, mixer_servo_settings);
  }
  return mixer_servo_ptr;
}

static hal::v5::optional_ptr<hal::actuator::rc_servo> door_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> door_servo()
{
  if (not door_servo_ptr) {
    auto servo_pca_ptr = pca();
    static auto door_pwm1 = servo_pca_ptr->get_pwm_channel<1>();
    constexpr hal::actuator::rc_servo::settings door_servo_settings{
      .frequency = 50,
      .min_angle = 0,
      .max_angle = 180,
      .min_microseconds = 750,
      .max_microseconds = 2250,
    };
    door_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), door_pwm1, door_servo_settings);
  }
  return door_servo_ptr;
}

static hal::v5::optional_ptr<hal::actuator::rc_servo> trap_door_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> trap_door_servo()
{
  if (not trap_door_servo_ptr) {
    static auto servo_pca_ptr = pca();
    static auto trap_door_pwm2 = servo_pca_ptr->get_pwm_channel<2>();
    constexpr hal::actuator::rc_servo::settings trap_door_servo_settings{
      .frequency = 50,
      .min_angle = 0,
      .max_angle = 180,
      .min_microseconds = 500,
      .max_microseconds = 2500,
    };
    trap_door_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), trap_door_pwm2, trap_door_servo_settings);
  }
  return trap_door_servo_ptr;
}

static hal::v5::optional_ptr<hal::actuator::rc_servo> arm_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> arm_servo()
{
  if (not arm_servo_ptr) {
    static auto servo_pca_ptr = pca();
    static auto arm_pwm3 = servo_pca_ptr->get_pwm_channel<3>();
    constexpr hal::actuator::rc_servo::settings arm_servo_settings{
      .frequency = 50,
      .min_angle = 0,
      .max_angle = 190,
      .min_microseconds = 600,
      .max_microseconds = 2400,
    };
    arm_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), arm_pwm3, arm_servo_settings);
  }
  return arm_servo_ptr;
}

static hal::v5::optional_ptr<hal::actuator::rc_servo> carousel_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> carousel_servo()
{
  if (not carousel_servo_ptr) {
    auto servo_pca_ptr = pca();
    static auto carousel_pwm4 = servo_pca_ptr->get_pwm_channel<4>();
    constexpr hal::actuator::rc_servo::settings carousel_servo_settings{
      .frequency = 50,
      .min_angle = 0,
      .max_angle = 190,
      .min_microseconds = 600,
      .max_microseconds = 2400,
    };
    carousel_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), carousel_pwm4, carousel_servo_settings);
  }
  return carousel_servo_ptr;
}

static hal::v5::optional_ptr<hal::actuator::rc_servo> cache_servo_ptr;
hal::v5::strong_ptr<hal::actuator::rc_servo> cache_servo()
{
  if (not cache_servo_ptr) {
    static auto servo_pca_ptr = pca();
    static auto cache_pwm5 = servo_pca_ptr->get_pwm_channel<5>();
    constexpr hal::actuator::rc_servo::settings cache_servo_settings{
      // WHAT IS CACHE SPECS
      .frequency = 50,         .min_angle = 0,           .max_angle = 190,
      .min_microseconds = 600, .max_microseconds = 2400,
    };
    cache_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
      driver_allocator(), cache_pwm5, cache_servo_settings);
  }
  return cache_servo_ptr;
}

static hal::v5::optional_ptr<hal::stm32f1::can_peripheral_manager_v2>
  can_manager;
static std::array<hal::v5::optional_ptr<hal::can_mask_filter>, 2> can_mask;
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
    auto f = hal::acquire_can_mask_filter(driver_allocator(), can_manager);
    hal::can_mask_filter::pair p;
    p.id = 0;
    p.mask = 0;
    can_mask[0] = f[0];
    can_mask[1] = f[1];
    can_mask.at(0)->allow(p);
  }
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

}  // namespace sjsu::science::resources
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
}  // namespace sjsu::science