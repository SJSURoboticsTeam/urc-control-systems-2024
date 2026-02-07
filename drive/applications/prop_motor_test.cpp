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
  for (uint8_t i = 0; i < sizeof(steer)/sizeof(steer[0]); i++) {
    steer[i]->feedback_request(
      hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float angle = steer[i]->feedback().angle();
    steer[i]->position_control(angle, 120);
  }
  hal::print(*console, "steer locked\n");

  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> prop[] = {
    resources::front_left_prop(),
    resources::front_right_prop(),
    resources::back_left_prop(),
    resources::back_right_prop()
  };
  hal::delay(*clock, 3s);
  hal::print(*console, "forward\n");
  float rpm = 20;
  for (uint8_t i = 0; i < sizeof(prop)/sizeof(prop[0]); i++) {
    if (i % 2) {
      prop[i]->velocity_control(rpm);
    } else {
      prop[i]->velocity_control(-rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "backward\n");
  for (uint8_t i = 0; i < sizeof(prop)/sizeof(prop[0]); i++) {
    if (i % 2) {
      prop[i]->velocity_control(-rpm);
    } else {
      prop[i]->velocity_control(rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "Fin\n");
  for (uint8_t i = 0; i < sizeof(prop)/sizeof(prop[0]); i++) {
    prop[i]->velocity_control(0);
  }
}
}  // namespace sjsu::drive
