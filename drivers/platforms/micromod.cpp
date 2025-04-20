#include <libhal-micromod/micromod.hpp>

#include "../hardware_map.hpp"
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/serial.hpp>

namespace sjsu::drivers {

application_framework initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();
  // static auto i2c = hal::lpc40::i2c(2);
  // static auto& counter = hal::micromod::v1::uptime_clock();
  // static auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

  // static auto& scl = hal::micromod::v1::output_g1();
  // static auto& sda = hal::micromod::v1::output_g0();
  // hal::print(terminal, "created g0 and g1\n");

  // static hal::bit_bang_i2c i2c({ .sda = &sda, .scl = &scl }, counter);
  // hal::print(terminal, "created i2c\n");

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
}  // namespace sjsu::drivers