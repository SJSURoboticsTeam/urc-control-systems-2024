#include "../applications/application.hpp"
#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>

using namespace hal::stm32f1;
namespace sjsu::arm {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;

  static uart uart1(hal::port<1>,
                    hal::buffer<1024>,
                    hal::serial::settings{
                      .baud_rate = 9600,
                    });
  static output_pin p_a_high('B', 3);  // INHC
  static output_pin p_b_high('B', 4);  // INHB

  static general_purpose_timer<peripheral::timer2> timer2;
  static pwm16_channel p_a_low =
    timer2.acquire_pwm16_channel(timer2_pin::pa0);  // INLB (A0)
  static advanced_timer<peripheral::timer1> timer1;
  static pwm16_channel p_b_low =
    timer1.acquire_pwm16_channel(timer1_pin::pa8);  // INLB (A0)

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  // static pwm16_channel p_a_low('A', 8);          // INLC
  // static pwm16_channel p_b_low('A', 0);          // INLB
  // hal::pwm16_channel* pwm_channel = nullptr;
  // hal::pwm_group_manager* pwm_frequency = nullptr;
  static output_pin led('C', 13);
  // auto cpu_frequency = frequency(peripheral::cpu);
  // static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  // maximum_speed_using_internal_oscillator();
  // release_jtag_pins();
  return {
    .led = &led,
    .console = &uart1,
    .clock = &steady_clock,
    .reset = +[]() { hal::cortex_m::reset(); },
    .a_low = &p_a_low,
    .b_low = &p_b_low,
    .a_high = &p_a_high,
    .b_high = &p_b_high,
  };
}
}  // namespace sjsu::arm