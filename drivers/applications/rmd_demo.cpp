// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <exception>

#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <libhal/units.hpp>
#include "../hardware_map.hpp"
namespace sjsu::drivers {
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_identifier_filter = resources::can_identifier_filter();

  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  can_bus_manager->baud_rate(1.0_MHz);

  hal::print(*console, "RMD MC-X Smart Servo Application Starting...\n\n");

  constexpr std::uint16_t starting_device_address = 0x140;
  std::uint16_t address_offset = 0;

  while (true) {
    try {
      auto const address = starting_device_address + address_offset;
      hal::print<32>(*console, "Using address: 0x%04X\n", address);
      hal::actuator::rmd_mc_x_v2 mc_x(
        *can_transceiver, *can_identifier_filter, *clock, 36.0f, address);

      auto print_feedback = [&mc_x, &console]() {
        mc_x.feedback_request(hal::actuator::rmd_mc_x_v2::read::status_2);
        mc_x.feedback_request(
          hal::actuator::rmd_mc_x_v2::read::multi_turns_angle);
        mc_x.feedback_request(
          hal::actuator::rmd_mc_x_v2::read::status_1_and_error_flags);

        hal::print<2048>(
          *console,
          "[%u] =================================\n"
          "raw_multi_turn_angle = %f\n"
          "raw_current = %d\n"
          "raw_speed = %d\n"
          "raw_volts = %d\n"
          "encoder = %d\n"
          "raw_motor_temperature = %d"
          "\n"
          "-------\n"
          "angle() = %f °deg\n"
          "current() = %f A\n"
          "speed() = %f rpm\n"
          "volts() = %f V\n"
          "temperature() = %f °C\n"
          "motor_stall() = %d\n"
          "low_pressure() = %d\n"
          "over_voltage() = %d\n"
          "over_current() = %d\n"
          "power_overrun() = %d\n"
          "speeding() = %d\n"
          "over_temperature() = %d\n"
          "encoder_calibration_error() = %d\n"
          "\n",

          mc_x.feedback().message_number,
          static_cast<float>(mc_x.feedback().raw_multi_turn_angle),
          mc_x.feedback().raw_current,
          mc_x.feedback().raw_speed,
          mc_x.feedback().raw_volts,
          mc_x.feedback().encoder,
          mc_x.feedback().raw_motor_temperature,
          mc_x.feedback().angle(),
          mc_x.feedback().current(),
          mc_x.feedback().speed(),
          mc_x.feedback().volts(),
          mc_x.feedback().temperature(),
          mc_x.feedback().motor_stall(),
          mc_x.feedback().low_pressure(),
          mc_x.feedback().over_voltage(),
          mc_x.feedback().over_current(),
          mc_x.feedback().power_overrun(),
          mc_x.feedback().speeding(),
          mc_x.feedback().over_temperature(),
          mc_x.feedback().encoder_calibration_error());
      };

      while (true) {
        mc_x.velocity_control(50.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        mc_x.velocity_control(0.0_rpm);
        hal::delay(*clock, 2000ms);
        print_feedback();

        mc_x.velocity_control(-50.0_rpm);
        hal::delay(*clock, 5000ms);
        print_feedback();

        mc_x.velocity_control(0.0_rpm);
        hal::delay(*clock, 2000ms);
        print_feedback();

        // Position control above 40 RPM seems to cause issues with position
        // control
        mc_x.position_control(0.0_deg, 30.0_rpm);
        hal::delay(*clock, 1s);
        print_feedback();

        mc_x.position_control(90.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(180.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(90.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(0.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(-45.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(-90.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(-45.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();

        mc_x.position_control(0.0_deg, 30.0_rpm);
        hal::delay(*clock, 2s);
        print_feedback();
      }
    } catch (hal::timed_out const&) {
      hal::print(
        *console,
        "hal::timed_out exception! which means that the device did not "
        "respond. Moving to the next device address in the list.\n");
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(*console, "hal::resource_unavailable_try_again\n");
      if (p_error.instance() == &can_transceiver) {
        hal::print(
          *console,
          "\n"
          "The CAN peripheral has received no acknowledgements from any other "
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
      hal::print(*console, "Unknown exception caught in (...) block\n");
      throw;
    }

    address_offset = (address_offset + 1) % 32;
    hal::delay(*clock, 1s);
  }
}
}  // namespace drivers
