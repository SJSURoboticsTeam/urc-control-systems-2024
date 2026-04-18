#include <drivetrain.hpp>
#include <drivetrain_math.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <resource_list.hpp>
#include <serial_commands.hpp>

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "\nappstart\n");
  constexpr hal::time_duration cycle_time = 250ms;
  constexpr sec cycle_time_sec = hal_time_duration_to_sec(cycle_time);
  auto mods_ptr = resources::swerve_modules();
  drivetrain dt(mods_ptr, cycle_time_sec);
  //TODO: some UB in the serial library maybe resulting in issues with the initialization of drivetrain

  std::array cmd_defs = {
    drivers::serial_commands::def{
      .m_prefix = "set-vel",
      .m_callback = [&](auto params) {
        if (params.size() != 3) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float x = drivers::serial_commands::parse_float(params[0]);
        float y = drivers::serial_commands::parse_float(params[1]);
        float w = drivers::serial_commands::parse_float(params[2]);
        hal::print<32>(*console, "Set vel: %f,%f,%f\n", x, y, w);
        dt.set_target_state({ { x, y }, w }, true);
      },
    },
    drivers::serial_commands::def{
      .m_prefix = "hard-home",
      .m_callback = [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        hal::print(*console, "Homing\n");
        dt.stop();
        dt.hard_home();
      },
    },
    drivers::serial_commands::def{
      .m_prefix = "print-actual-states",
      .m_callback = [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        auto mods = resources::swerve_modules();
        hal::print(*console, "Actual States\n");
        for (uint8_t i = 0; i < mods->size(); i++) {
          auto state = mods->at(i)->get_actual_state_cache();
          hal::print<64>(*console,
                         "mod[%i]:(%f,%f)\n",
                         i,
                         state.steer_angle,
                         state.propulsion_velocity);
        }
      },
    },
    drivers::serial_commands::def{
      .m_prefix = "print-target-states",
      .m_callback = [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        auto mods = resources::swerve_modules();
        hal::print(*console, "Target States\n");
        for (uint8_t i = 0; i < mods->size(); i++) {
          auto state = mods->at(i)->get_target_state();
          hal::print<64>(*console,
                         "mod[%i]:(%f,%f)\n",
                         i,
                         state.steer_angle,
                         state.propulsion_velocity);
        }
      },
    }
  };

  drivers::serial_commands::handler cmd{ console };

  while (true) {
    cmd.handle(cmd_defs);
    try {
    } catch (hal::exception const& e) {
      switch (e.error_code()) {
        case std::errc::argument_out_of_domain:
          hal::print(*console, "Error: invalid argument length or type\n");
          break;
        default:
          hal::print<32>(*console, "Error code: %d\n", e.error_code());
          break;
      }
    }
    dt.periodic();
    try {
      dt.periodic();
    } catch (hal::exception e) {
      hal::print(*console, "\nexpection thown in periodic\n");
      throw;
    }
  }
}
}  // namespace sjsu::drive
