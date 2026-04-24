#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/motor.hpp>
#include <libhal/servo.hpp>
#include <resource_list.hpp>

// TODO: resource file must be updated to return abstract interface types
//       (velocity_servo for steer, velocity_motor for prop) before this file will compile

namespace sjsu::drive {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "app starting\n");
  // TODO: resource file must return hal::velocity_servo instead of rmd_mc_x_v2
  hal::v5::strong_ptr<hal::velocity_servo> steer_motor_array[] = {
    resources::front_left_steer(),
    resources::front_right_steer(),
    resources::back_left_steer(),
    resources::back_right_steer()
  };
  std::span steer_motors = { steer_motor_array };

  // configure steer speed then lock to current position
  for (uint8_t i = 0; i < steer_motors.size(); i++) {
    steer_motors[i]->configure({ .velocity = 120 });
    float angle = steer_motors[i]->position();
    steer_motors[i]->position(angle);
  }
  hal::print(*console, "steer locked\n");

  // TODO: resource file must return hal::velocity_motor instead of rmd_mc_x_v2
  hal::v5::strong_ptr<hal::velocity_motor> prop_motor_array[] = {
    resources::front_left_prop(),
    resources::front_right_prop(),
    resources::back_left_prop(),
    resources::back_right_prop()
  };
  std::span prop_motors{ prop_motor_array };
  hal::delay(*clock, 3s);
  hal::print(*console, "forward\n");
  float rpm = 20;
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    if (i % 2) {
      prop_motors[i]->drive(rpm);
    } else {
      prop_motors[i]->drive(-rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "backward\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    if (i % 2) {
      prop_motors[i]->drive(-rpm);
    } else {
      prop_motors[i]->drive(rpm);
    }
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "Fin\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    prop_motors[i]->drive(0);
  }
}
}  // namespace sjsu::drive