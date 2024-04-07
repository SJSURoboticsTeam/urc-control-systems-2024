#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/pwm.hpp>
// #include <libhal-lpc40/system_controller.hpp> //not sure why we need this?
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include <libhal-util/units.hpp>

#include "../hardware_map.hpp"
namespace sjsu::science {

application_framework initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // Set the MCU to the maximum clock speed
  hal::lpc40::maximum(12.0_MHz);

  auto cpu_frequency = hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu);
  static hal::cortex_m::dwt_counter counter(cpu_frequency);

  // Serial
  static std::array<hal::byte, 1024> recieve_buffer0{};
  static hal::lpc40::uart uart0(0,
                                recieve_buffer0,
                                hal::serial::settings{
                                  .baud_rate = 38400,
                                });
  // Don't think we need can for the science applications thus far
  static hal::can::settings can_settings{ .baud_rate = 1.0_MHz };
  static auto can = (hal::lpc40::can::get(2, can_settings));

  static hal::lpc40::input_pin in_pin0(1, 15, hal::input_pin::settings{});
  static hal::lpc40::input_pin in_pin1(1, 23, hal::input_pin::settings{});
  static hal::lpc40::input_pin in_pin2(1, 22, hal::input_pin::settings{});

  static hal::lpc40::pwm pwm_1_6(1, 6);
  static hal::lpc40::pwm pwm_1_5(1, 5);

  static hal::lpc40::adc adc_4(4);
  static hal::lpc40::adc adc_5(5);

  std::array<hal::byte, 1024> receive_buffer{};
  static hal::lpc40::uart uart1(0,
                                receive_buffer,
                                {
                                  .baud_rate = 115200.0f,
                                });

  static hal::lpc40::i2c i2c(2);

  return application_framework{
    .terminal = &uart0,
    .can = &can,
    .in_pin0 = &in_pin0,
    .in_pin1 = &in_pin1,
    .in_pin2 = &in_pin2,
    .pwm_1_6 = &pwm_1_6,
    .pwm_1_5 = &pwm_1_5,
    .adc_4 = &adc_4,
    .adc_5 = &adc_5,
    .esp = &uart1,
    .i2c = &i2c,
    .steady_clock = &counter,
    .reset = []() { hal::cortex_m::reset(); },
  };
};
}  // namespace sjsu::science
