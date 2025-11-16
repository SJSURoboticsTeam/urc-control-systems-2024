#include "../hardware_map.hpp"
#include <serial_commands.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>

namespace sjsu::drivers {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();

  // reduce motor commands
  // add commands to toggle LED lighting

  bool flashing = true;

  std::array cmd_defs = {
    // not going to include all variations of {F/B/D}{L/R}{S/P}, they work in
    // the same fashion
    drivers::serial_commands::def{
      "fls",
      [&console](auto params) {
        if (params.size() != 1) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float speed = drivers::serial_commands::parse_float(params[0]);
        hal::print<32>(*console, "Set FLS: %f\n", speed);
      },
    },
    drivers::serial_commands::def{
      "toggle-flash",
      [&console, &flashing](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        flashing = !flashing;
        hal::print(*console, "Toggled flashing.\n");
      },
    },
    drivers::serial_commands::def{
      "noparam",
      [&console](auto params) {
        if (params.size() != 0) {
          throw hal::argument_out_of_domain(nullptr);
        }
        hal::print(*console, "Executed no parameter command\n");
      },
    },
    drivers::serial_commands::def{
      "multiparam",
      [&console](auto params) {
        if (params.size() != 3) {
          throw hal::argument_out_of_domain(nullptr);
        }
        float p1 = drivers::serial_commands::parse_float(params[0]);
        int p2 = drivers::serial_commands::parse_int(params[1]);
        int p3 = drivers::serial_commands::parse_int(params[2]);
        hal::print<64>(
          *console, "Multi-parameter command: %f %d %d\n", p1, p2, p3);
      },
    },
  };  // namespace sjsu::drive

  bool led_stat = true;

  drivers::serial_commands::handler cmd{ console };

  while (true) {
    // we tradeoff responsiveness for performance, effectively batching a bunch
    // of reads in a single 500ms interval into single read at the end of a
    // 500ms time block
    //
    // to ensure that the client doesn't overflow the receive buffer in this
    // time, we echo every character the client sends so the client can wait
    // until we "ack" their command.
    //
    // this makes it awkward if your TTY client sends one character at a time
    // instead of an entire line at a time, in these cases it makes more sense
    // to simply reconfigure your TTY client
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

    // Toggle LED
    led->level(led_stat);
    hal::delay(*clock, 500ms);
    if (flashing) {
      led_stat = !led_stat;
    }
  }
}
}  // namespace sjsu::drivers
