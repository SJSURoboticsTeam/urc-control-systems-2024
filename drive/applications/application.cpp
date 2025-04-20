#include "./application.hpp"

#include "drive_configuration_updater.hpp"
#include "homing.hpp"
#include "settings.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

namespace sjsu::drive {

void application(hardware_map_t& hardware_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *hardware_map.clock.value();
  auto& console = *hardware_map.terminal.value();
  auto& can_transceiver = *hardware_map.can_transceiver.value();
  // auto& can_bus_manager = *hardware_map.can_bus_manager.value();
  // auto& can_identifier_filter = *hardware_map.can_identifier_filter.value();

  // can_bus_manager.baud_rate(1.0_MHz);
  hal::can_message_finder spin_reader(can_transceiver, 0x101);
  hal::can_message_finder drive_reader(can_transceiver, 0x102);
  hal::can_message_finder translate_reader(can_transceiver, 0x103);
  hal::can_message_finder speed_reader(can_transceiver, 0x104);
  hal::can_message_finder homing_reader(can_transceiver, 0x105);

  // const hal::u8 system_reset = 0x76;
  hal::print(console, "created things that we need.\n");

  auto& steering_modules = *hardware_map.steering_modules;
  auto& start_wheel_settings = *hardware_map.start_wheel_setting_span;
  // using namespace std::chrono_literals;
  // using namespace hal::literals;

  // auto& steering = *hardware_map.steering;
  // // auto& mission_control = *p_framework.mc;
  // auto& terminal = *hardware_map.terminal.value();
  // auto& clock = *hardware_map.clock.value();

  // auto& router = *hardware_map.router;

  // drive_configuration_updater configuration_updater;

  // configuration_updater.set_sensitivity(config_sensitivity);
  // configuration_updater.set_max_rate(config_max_delta);

  // hal::delay(clock, 1000ms);
  // hal::print(terminal, "Starting control loop...");

  // // float next_update = static_cast<float>(clock.uptime()) /
  // clock.frequency() + 5; float then = static_cast<float>(clock.uptime()) /
  // clock.frequency(); while (true) {
  //   // Calculate time since last frame. Use this for physics.
  //   float now = static_cast<float>(clock.uptime()) / clock.frequency();
  //   float dt = now - then;
  //   then = now;

  //   // if (next_update < now) {
  //   //   // // Time out in 10 ms
  //   //   // auto timeout = hal::create_timeout(clock, 1s);
  //   //   // auto commands = mission_control.get_command(timeout).value();

  //   //   // Create a new target and set the updater to go to the new target
  //   //   drive_configuration target;
  //   //   target.steering_angle = commands.steering_angle;
  //   //   target.wheel_heading = commands.wheel_heading;
  //   //   target.wheel_speed = commands.wheel_speed;

  //   //   // Validate the target
  //   //   target = validate_configuration(target);

  //   //   configuration_updater.set_target(target);

  //   //   // Next update from mission control in 100 ms (0.1 s)
  //   //   float now = static_cast<float>(clock.uptime().ticks) /
  //   clock.frequency().operating_frequency;
  //   //   next_update = now + 0.1;
  //   // }

  //   // Update the configuration
  //   configuration_updater.update(dt);
  //   // Get the current configuration
  //   drive_configuration current_configuration =
  //   configuration_updater.get_current();

  //   // Calculate the turning radius
  //   float turning_radius = 1 / std::tan(current_configuration.steering_angle
  //   * std::numbers::pi / 180);
  //   // Calculate the current wheel settings.
  //   auto wheel_settings = steering.calculate_wheel_settings(turning_radius,
  //   current_configuration.wheel_heading, current_configuration.wheel_speed /
  //   100);

  //   // Move all the wheels
  //   router.move(wheel_settings);

  // static hal::actuator::rmd_mc_x_v2 mc_x_front_right_steer(
  //   can_transceiver,
  //   can_identifier_filter,
  //   clock,
  //   start_wheel_settings[0].geer_ratio,
  //   start_wheel_settings[0].steer_id);
  // hal::print(console, "rmd\n");

  // static steering_module front_right_leg = {
  //   .steer = &mc_x_front_right_steer,
  //   .propulsion = nullptr
  //   // .propulsion = &front_right_prop,
  // };

  // static std::array<steering_module, 1> steering_modules_arr = {
  //   front_right_leg
  // };

  // static std::span<steering_module, 1> steering_modules_span =
  //   steering_modules_arr;

  //  hal::print(console, "homing\n");
  // home(steering_modules, start_wheel_settings, can_transceiver, clock,
  // console); hal::delay(clock, 1000ms);
  /**
   * 101,102,103, 104, 105, 148+16^2..steer id + 16^2
   */
   
  home(steering_modules, start_wheel_settings, clock, console);
  while (false) {
    try {
      std::optional<hal::can_message> msg = homing_reader.find();

      if (msg) {
        hal::print(console, "found message\n");
        home(steering_modules, start_wheel_settings, clock, console);

        hal::print(console, "Done homing\n");
      }
      hal::print<128>(console,
                      "Circular Buffer Size: %d\n",
                      can_transceiver.receive_cursor());
    } catch (hal::timed_out const&) {
      hal::print(
        console,
        "hal::timed_out exception! which means that the device did not "
        "respond. Moving to the next device address in the list.\n");
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(console, "hal::resource_unavailable_try_again\n");
      if (p_error.instance() == &can_transceiver) {
        hal::print(
          console,
          "\n"
          "device on the bus. It appears as if the peripheral is not connected "
          "to a can network. This can happen if the baud rate is incorrect, "
          "the CAN transceiver is not functioning, or the devices on the bus "
          "are not responding."
          "\n"
          "Calling terminate!"
          "\n"
          "Consider powering down the system and checking all of your "
          "connections before restarting the application.");
        std::terminate();
      }
      // otherwise keep trying with other addresses
    } catch (...) {
      hal::print(console, "Unknown exception caught in (...) block\n");
      throw;  // see if anyone else can handle the exception
    }

    // address_offset = (address_offset + 1) % 16;
    hal::delay(clock, 1s);
  }
}

}  // namespace sjsu::drive
