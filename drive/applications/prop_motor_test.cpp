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
  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> steer_motor_array[] = {
    resources::front_left_steer(),
    resources::front_right_steer(),
    resources::back_left_steer(),
    resources::back_right_steer()
  };
  std::span steer_motors = {steer_motor_array};
  for (uint8_t i = 0; i < steer_motors.size(); i++) {
    steer_motors[i]->feedback_request(
      hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
    float angle = steer_motors[i]->feedback().angle();
    steer_motors[i]->position_control(angle, 120);
  }
  hal::print(*console, "steer locked\n");

  hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> prop_motor_array[] = {
    resources::front_left_prop(),
    resources::front_right_prop(),
    resources::back_left_prop(),
    resources::back_right_prop()
  };
  std::span prop_motors{prop_motor_array};
  hal::delay(*clock, 3s);
  hal::print(*console, "forward\n");
  float rpm = 20;
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    if (i % 2) {
      prop_motors[i]->velocity_control(rpm);
    } else {
      prop_motors[i]->velocity_control(-rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "backward\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    if (i % 2) {
      prop_motors[i]->velocity_control(-rpm);
    } else {
      prop_motors[i]->velocity_control(rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "Fin\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    prop_motors[i]->velocity_control(0);
  }
}
}  // namespace sjsu::drive
