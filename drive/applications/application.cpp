#include "./application.hpp"

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include "drive_configuration_updater.hpp"

#include "settings.hpp"

namespace sjsu::drive {

void application(hardware_map_t& hardware_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& steering = *hardware_map.steering;
  // auto& mission_control = *p_framework.mc;
  auto& terminal = *hardware_map.terminal.value();
  auto& clock = *hardware_map.clock.value();

  auto& router = *hardware_map.router;


  drive_configuration_updater configuration_updater;

  configuration_updater.set_sensitivity(config_sensitivity);
  configuration_updater.set_max_rate(config_max_delta);


  hal::delay(clock, 1000ms);
  hal::print(terminal, "Starting control loop...");

  // float next_update = static_cast<float>(clock.uptime()) / clock.frequency() + 5;
  float then = static_cast<float>(clock.uptime()) / clock.frequency();
  while (true) {
    // Calculate time since last frame. Use this for physics.
    float now = static_cast<float>(clock.uptime()) / clock.frequency();
    float dt = now - then;
    then = now;

    // if (next_update < now) {
    //   // // Time out in 10 ms
    //   // auto timeout = hal::create_timeout(clock, 1s);
    //   // auto commands = mission_control.get_command(timeout).value();

    //   // Create a new target and set the updater to go to the new target
    //   drive_configuration target;
    //   target.steering_angle = commands.steering_angle;
    //   target.wheel_heading = commands.wheel_heading;
    //   target.wheel_speed = commands.wheel_speed;

    //   // Validate the target
    //   target = validate_configuration(target);
      
    //   configuration_updater.set_target(target);

    //   // Next update from mission control in 100 ms (0.1 s)
    //   float now = static_cast<float>(clock.uptime().ticks) / clock.frequency().operating_frequency;
    //   next_update = now + 0.1;
    // }

    // Update the configuration
    configuration_updater.update(dt);
    // Get the current configuration
    drive_configuration current_configuration = configuration_updater.get_current();
    
    // Calculate the turning radius
    float turning_radius = 1 / std::tan(current_configuration.steering_angle * std::numbers::pi / 180);
    // Calculate the current wheel settings.
    auto wheel_settings = steering.calculate_wheel_settings(turning_radius, current_configuration.wheel_heading, current_configuration.wheel_speed / 100);

    // Move all the wheels
    router.move(wheel_settings);
  }





}

}  // namespace sjsu::science
