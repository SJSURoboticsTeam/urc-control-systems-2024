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
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>


#include "../resource_list.hpp"
#include <memory_resource>


namespace resources {
using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::array<hal::byte, 1024> driver_memory{};
std::pmr::monotonic_buffer_resource resource(driver_memory.data(),
                                             driver_memory.size(),
                                             std::pmr::null_memory_resource());

std::pmr::polymorphic_allocator<> driver_allocator()
{
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

hal::v5::strong_ptr<hal::adc> adc()
{
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
  return hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb0);
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

hal::v5::strong_ptr<hal::spi> spi()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::spi>(driver_allocator(),
                                                     hal::bus<1>,
                                                     hal::spi::settings{
                                                       .clock_rate = 250.0_kHz,
                                                       .clock_polarity = false,
                                                       .clock_phase = false,
                                                     });
}

hal::v5::strong_ptr<hal::output_pin> spi_chip_select()
{
  return hal::v5::make_strong_ptr<decltype(gpio_a().acquire_output_pin(4))>(
    driver_allocator(), gpio_a().acquire_output_pin(4));
}

hal::v5::strong_ptr<hal::input_pin> input_pin()
{
  return hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(4))>(
    driver_allocator(), gpio_b().acquire_input_pin(4));
}

auto& timer1()
{
  static hal::stm32f1::advanced_timer<st_peripheral::timer1> timer1{};
  return timer1;
}

hal::v5::strong_ptr<hal::timer> timed_interrupt()
{
#if 0
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer2> timer2;
  auto timer_callback_timer = timer2.acquire_timer();
  return hal::v5::make_strong_ptr<decltype(timer_callback_timer)>(
    driver_allocator(), std::move(timer_callback_timer));
#endif
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  static auto timer_old_pwm =
    timer1().acquire_pwm(hal::stm32f1::timer1_pin::pa8);
  return hal::v5::make_strong_ptr<decltype(timer_old_pwm)>(
    driver_allocator(), std::move(timer_old_pwm));
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel()
{
  auto timer_pwm_channel =
    timer1().acquire_pwm16_channel(hal::stm32f1::timer1_pin::pa8);
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

hal::v5::strong_ptr<hal::interrupt_pin> interrupt_pin()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::stream_dac_u8> stream_dac()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::dac> dac()
{
  throw hal::operation_not_supported(nullptr);
}

// Watchdog implementation using global function pattern from original
class stm32f103c8_watchdog : public custom::watchdog
{
public:
  void start() override
  {
    m_stm_watchdog.start();
  }

  void reset() override
  {
    m_stm_watchdog.reset();
  }

  void set_countdown_time(hal::time_duration p_wait_time) override
  {
    m_stm_watchdog.set_countdown_time(p_wait_time);
  }

  bool check_flag() override
  {
    return m_stm_watchdog.check_flag();
  }

  void clear_flag() override
  {
    m_stm_watchdog.clear_flag();
  }

private:
  hal::stm32f1::independent_watchdog m_stm_watchdog{};
};

hal::v5::strong_ptr<custom::watchdog> watchdog()
{
  return hal::v5::make_strong_ptr<stm32f103c8_watchdog>(driver_allocator());
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

}  // namespace drive::resources

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