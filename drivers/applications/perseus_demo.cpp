#include <perseus_bldc.hpp>
#include <serial_commands.hpp>
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>
#include <can_util.hpp>


namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  hal::print(*console, "App Start\n");
  auto clock = resources::clock();
  auto can = resources::can_transceiver();
  perseus_bldc perseus(can,clock,0x03);
  hal::print(*console, "Motor Made\n");
  
  while (true) {
    hal::print(*console, "Loop Start\n");
    hal::print<64>(
    *console, "position: %f\n", perseus.get_position());
    perseus.set_power(0.3);
    hal::print(*console, "Switch 0.3\n");
    hal::delay(*clock, 1s);
    perseus.set_power(-0.3);
    hal::print(*console, "Switch -0.3\n");
    hal::delay(*clock, 1s);
  }
}
}  // namespace sjsu::drivers