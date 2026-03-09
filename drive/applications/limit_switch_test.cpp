#include <drivetrain_math.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::v5::strong_ptr<hal::input_pin> switches[] = {
    resources::front_left_limit_switch(),
    resources::front_right_limit_switch(),
    resources::back_left_limit_switch(),
    resources::back_right_limit_switch(),
  };
  while (true) {
    for (unsigned int i = 0; i < sizeof(switches) / sizeof(switches[0]); i++) {
      hal::print<32>(*console, "%d", switches[i]->level());
    }
    hal::print<32>(*console, "\n");
    hal::delay(*clock, 100ms);
  }
}
}  // namespace sjsu::drive
