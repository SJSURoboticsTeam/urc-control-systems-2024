#include <libhal-micromod/micromod.hpp>

#include "../hardware_map.hpp"

namespace sjsu::drivers {

application_framework initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();

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
    .pwm0 = &hal::micromod::v1::pwm0(),
    .pwm1 = &hal::micromod::v1::pwm1(),
    .adc0 = &hal::micromod::v1::a0(),
    .adc1 = &hal::micromod::v1::a1(),
    // .esp,
    .i2c = &hal::micromod::v1::i2c(),
    .steady_clock = &hal::micromod::v1::uptime_clock(),
    .reset = +[]() { hal::micromod::v1::reset(); },
  };
}
}