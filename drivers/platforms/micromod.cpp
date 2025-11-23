#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-micromod/micromod.hpp>

#include <resource_list.hpp>

namespace sjsu::drivers {

application_framework initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();

  // for stm till pwm gets added
  using st_peripheral = hal::stm32f1::peripheral;
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer2> timer2{};
  static auto timer_pwm_channel_0 =
    timer2.acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa0);
  static auto timer_pwm_channel_1 =
    timer2.acquire_pwm16_channel(hal::stm32f1::timer2_pin::pa1);

  return {
    .terminal = &hal::micromod::v1::console(hal::buffer<128>),
    .can = &hal::micromod::v1::can(),
    .in_pin0 = &hal::micromod::v1::input_g0(),
    .in_pin1 = &hal::micromod::v1::input_g1(),
    .in_pin2 = &hal::micromod::v1::input_g2(),
    .led = &hal::micromod::v1::led(),
    .out_pin0 = &hal::micromod::v1::output_g0(),
    .out_pin1 = &hal::micromod::v1::output_g1(),
    .out_pin2 = &hal::micromod::v1::output_g2(),
    .out_pin3 = &hal::micromod::v1::output_g3(),
    .out_pin4 = &hal::micromod::v1::output_g4(),
    .pwm0 = &timer_pwm_channel_0,
    .pwm1 = &timer_pwm_channel_1,
    // .adc0 = &hal::micromod::v1::a0(),
    // .adc1 = &hal::micromod::v1::a1(),
    // .esp,
    .i2c = &hal::micromod::v1::i2c(),
    .steady_clock = &hal::micromod::v1::uptime_clock(),
    .reset = +[]() { hal::micromod::v1::reset(); },
  };
}
}  // namespace sjsu::drivers