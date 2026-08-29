#include <can_util.hpp>
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <perseus_bldc.hpp>
#include <resource_list.hpp>
#include <serial_commands.hpp>

namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  hal::print(*console, "App Start\n");
  auto clock = resources::clock();
  auto can = resources::can_transceiver();
  perseus_bldc perseus(can, clock, 0x03);
  hal::print(*console, "Motor Made\n");

  std::array cmd_defs = {
    drivers::serial_commands::def{
      "power",
      [&](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float p1 = drivers::serial_commands::parse_float(params[0]);
        p1 = std::clamp(p1, -0.9f, 0.9f);
        perseus.set_power(p1);
      },
    },
    drivers::serial_commands::def{
      "read_pos",
      [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        hal::print<64>(*console, "Postion:%f\n", perseus.get_position());
      },
    },
    drivers::serial_commands::def{
      "home",
      [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        perseus.home();
      },
    },
    drivers::serial_commands::def{
      "is_homing",
      [&](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        hal::print<64>(*console, "Is Homing::%d\n", perseus.is_homing());
      },
    },
    drivers::serial_commands::def{
      "set_pos_target",
      [&](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float p1 = drivers::serial_commands::parse_float(params[0]);
        perseus.set_target_position(p1);
      },
    },
  };
  drivers::serial_commands::handler cmd(console);
  while (true) {
    try {
      cmd.handle(cmd_defs);
    } catch (hal::exception e) {
      switch (e.error_code()) {
        case std::errc::argument_out_of_domain:
          hal::print(*console, "Error: invalid argument length or type\n");
          break;
        default:
          hal::print<32>(*console, "Error code: %d\n", e.error_code());
          break;
      }
    }
  }
}
}  // namespace sjsu::drivers
