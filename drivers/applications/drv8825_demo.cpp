#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include <drv8825.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application()
{
  // configure drivers
  auto clock = resources::clock();
  auto terminal = resources::console();
  hal::print(*terminal, "console initialized\n");
  auto dir_pin = resources::output_pin_6();
  hal::print(*terminal, "dir pin initialized\n");
  auto step_pin = resources::output_pin_4();
  hal::print(*terminal, "step pin initialized\n");
  auto m1_pin = resources::output_pin_5();
  hal::print(*terminal, "m0 pin initialized\n");
  auto m2_pin = resources::output_pin_0();
  hal::print(*terminal, "m1 pin initialized\n");
  auto m3_pin = resources::output_pin_7();
  hal::print(*terminal, "m2 pin initialized\n");

  // drv8825 stepper_controller(
  //   drv8825::ctor_params{ .direction_pin = dir_pin,
  //                         .step_pin = step_pin,
  //                         .steady_clock = clock,
  //                         .motor_step_factor = drv8825::step_factor::one,
  //                         .full_steps_per_rotation = 2048,
  //                         .step_half_period = 450us,
  //                         .mode_pins = { m1_pin, m2_pin, m3_pin } });  // TODO

  // hal::print(*terminal, "starting motor\n");

  // while (true) {
  //   stepper_controller.step(2048);
  //   hal::delay(*clock, 1s);
  //   stepper_controller.step(-2048);
  //   hal::delay(*clock, 1s);
  // }
}
}  // namespace sjsu::drivers