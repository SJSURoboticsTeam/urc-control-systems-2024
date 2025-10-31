#include "../hardware_map.hpp"
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <perseus_bldc.hpp>

namespace sjsu::arm::resources {
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
        1'000'000, // 1 MHz
        *clock_ref,
        std::chrono::milliseconds(1),
        hal::stm32f1::can_pins::pb9_pb8);
  }
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

arm_can_finders can_finders(
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::u16 home,
  hal::u16 arm,
  hal::u16 endeffector)
{
  return { .home_finder = hal::v5::make_strong_ptr<hal::can_message_finder>(
             driver_allocator(),
             *transceiver,
             home),  // finds message with address for home
           .arm_finder = hal::v5::make_strong_ptr<hal::can_message_finder>(
             driver_allocator(), *transceiver, arm),
           .endeffector_finder =
             hal::v5::make_strong_ptr<hal::can_message_finder>(
               driver_allocator(), *transceiver, endeffector) };
}

hal::v5::strong_ptr<arm_joints> arm_servos(hal::v5::strong_ptr<hal::can_transceiver> transceiver)
{
  auto idf1 = hal::acquire_can_identifier_filter(driver_allocator(), can_manager); // each filter returns 4 ids
  auto idf2 =
    hal::acquire_can_identifier_filter(driver_allocator(), can_manager);
  arm_joints arm_servos = {
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // track
      driver_allocator(),
      transceiver,
      idf1[0],
      clock(),
      753,
      0x120),
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // shoulder
      driver_allocator(),
      transceiver,
      idf1[1],
      clock(),
      753,
      0x121),
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // elbow
      driver_allocator(),
      transceiver,
      idf1[2],
      clock(),
      753,
      0x122),
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // wrist pitch 1
      driver_allocator(),
      transceiver,
      idf1[3],
      clock(),
      753,
      0x123),
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // wrist pitch 2
      driver_allocator(),
      transceiver,
      idf2[0],
      clock(),
      753,
      0x124),
    hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(  // clamp
      driver_allocator(),
      transceiver,
      idf2[1],
      clock(),
      753,
      0x125)
  };
  return hal::make_strong_ptr<arm_joints>(driver_allocator(), arm_servos);
}

hal::v5::strong_ptr<limit_pins> arm_home_pins()
{
  limit_pins pins = {
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(0))>(
      driver_allocator(), gpio_a().acquire_input_pin(0)),
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(15))>(
      driver_allocator(), gpio_a().acquire_input_pin(15)),
    hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(3))>(
      driver_allocator(), gpio_b().acquire_input_pin(3)),  // elbow
    hal::v5::make_strong_ptr<decltype(gpio_b().acquire_input_pin(12))>(
      driver_allocator(), gpio_b().acquire_input_pin(12)),  // wrist pitch
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(4))>(
      driver_allocator(), gpio_a().acquire_input_pin(4)),  // wrist roll
    hal::v5::make_strong_ptr<decltype(gpio_a().acquire_input_pin(5))>(
      driver_allocator(), gpio_a().acquire_input_pin(5))  // clamp
  };
  return hal::make_strong_ptr<limit_pins>(driver_allocator(), pins);
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

}  // namespace sjsu::arm::resources
namespace sjsu::arm {
void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(sjsu::arm::resources::terminate_handler);
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
  // hal::stm32f1::activate_mco_pa8(
  //   hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();
}
}  // namespace sjsu::arm