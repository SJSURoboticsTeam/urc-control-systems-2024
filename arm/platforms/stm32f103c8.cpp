#include "../hardware_map.hpp"
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
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

auto& get_can_peripheral()
{
  using namespace std::chrono_literals;
  auto clock = resources::clock();
  static hal::stm32f1::can_peripheral_manager can(
    100_kHz,
    *clock,
    1ms,
    hal::stm32f1::can_pins::pb9_pb8);  // this needs to be static because we are
                                       // returning a non strong pointer type
  return can;
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver(
  std::span<hal::can_message> receive_buffer)
{
  auto transceiver = get_can_peripheral().acquire_transceiver(receive_buffer);
  return hal::v5::make_strong_ptr<decltype(transceiver)>(
    driver_allocator(), std::move(transceiver));
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  auto bus_man = get_can_peripheral().acquire_bus_manager();
  return hal::v5::make_strong_ptr<decltype(bus_man)>(driver_allocator(),
                                                     std::move(bus_man));
}

template<hal::u8 set_number>
auto& get_identifier_filter_set()
{
  static auto filter_set = get_can_peripheral().acquire_identifier_filter();
  return filter_set;
}

arm_can_finders can_finders(
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::u16 home,
  hal::u16 arm,
  hal::u16 endeffector)
{
  return { .home_finder = hal::v5::make_strong_ptr<hal::can_message_finder>(
             driver_allocator(), *transceiver, home), // finds message with address for home
           .arm_finder = hal::v5::make_strong_ptr<hal::can_message_finder>(
             driver_allocator(), *transceiver, arm),
           .endeffector_finder =
             hal::v5::make_strong_ptr<hal::can_message_finder>(
               driver_allocator(), *transceiver, endeffector) };
}

arm_joints arm_servos(hal::v5::strong_ptr<hal::can_transceiver> transceiver)
{
  return
  {
    .track_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,  // this will allow the class to send messages to the actual
                    // perseus controller
      get_identifier_filter_set<0>().filter[0],
      clock(),
      100,
      0x120),
    .shoulder_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,
      get_identifier_filter_set<0>().filter[1],
      clock(),
      100,
      0x121),
    .elbow_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,
      get_identifier_filter_set<0>().filter[2],
      clock(),
      100,
      0x122),
    .wrist_pitch_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,
      get_identifier_filter_set<1>().filter[0],
      clock(),
      100,
      0x123),
    .wrist_roll_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,
      get_identifier_filter_set<1>().filter[1],
      clock(),
      100,
      0x124),
    .clamp_servo = hal::v5::make_strong_ptr<sjsu::drivers::perseus_bldc>(
      driver_allocator(),
      transceiver,
      get_identifier_filter_set<1>().filter[1],
      clock(),
      100,
      0x125)
  };
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
  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();
}
}  // namespace sjsu::arm