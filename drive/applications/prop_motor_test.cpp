#include <array>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/motor.hpp>
#include <libhal/servo.hpp>
#include <propulsion_controller.hpp>
#include <resource_list.hpp>
#include <steer_controller.hpp>

namespace sjsu::drive {
void application()
{

  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "app starting\n");
  bool constexpr lock_steer = false;
  if (lock_steer) {
    std::array steer_motors = { resources::front_left_steer(),
                                resources::front_right_steer(),
                                resources::back_left_steer(),
                                resources::back_right_steer() };

    // configure steer speed then lock to current position
    for (uint8_t i = 0; i < steer_motors.size(); i++) {
      steer_motors.at(i)->stop();
    }
    hal::print(*console, "steer locked\n");
  }

  std::array prop_motors = { resources::front_left_prop(),
                             resources::front_right_prop(),
                             resources::back_left_prop(),
                             resources::back_right_prop() };
  // hal::delay(*clock, 3s);
  hal::print(*console, "forward\n");
  float rpm = 200;
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    prop_motors.at(i)->set_target_velocity(rpm);
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "backward\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    prop_motors.at(i)->set_target_velocity(-rpm);
  }
  hal::delay(*clock, 8s);
  hal::print(*console, "Fin\n");
  for (uint8_t i = 0; i < prop_motors.size(); i++) {
    prop_motors.at(i)->set_target_velocity(0);
  }
}
}  // namespace sjsu::drive
