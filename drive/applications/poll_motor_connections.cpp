#include <drivetrain_math.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "app starting\n");
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> steer[] = {
    resources::front_left_steer(),
    resources::front_right_steer(),
    resources::back_left_steer(),
    resources::back_right_steer()
  };
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> prop[] = {
    resources::front_left_prop(),
    resources::front_right_prop(),
    resources::back_left_prop(),
    resources::back_right_prop()
  };
  while (true) {
    for (unsigned int i = 0; i < sizeof(steer) / sizeof(steer[0]); i++) {
      try {
        steer[i]->feedback_request(
          hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
      } catch (hal::operation_not_supported e) {
        hal::print<64>(
          *console, "steer[%d], operation_not_supported: %d\n", i, e.error_code());
      } catch (hal::timed_out e) {
        hal::print<64>(
          *console, "steer[%d], timed_out: %d\n", i, e.error_code());
      } catch (hal::exception e) {
        hal::print<64>(
          *console, "steer[%d], error code: %d\n", i, e.error_code());
      }
    }
    for (unsigned int i = 0; i < sizeof(prop) / sizeof(prop[0]); i++) {
      try {
        prop[i]->feedback_request(
          hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
      } catch (hal::exception e) {
        hal::print<64>(
          *console, "prop[%d], error code: %d\n", i, e.error_code());
      }
    }
    hal::delay(*clock, 250ms);
  }
}
}  // namespace sjsu::drive
