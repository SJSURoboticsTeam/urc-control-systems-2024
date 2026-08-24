#include <can_util.hpp>
#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

namespace sjsu::drivers {
void application()
{
  using namespace std::chrono_literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto can = resources::can_transceiver();
  auto current_pos = can->receive_cursor();
  auto can_transceiver = resources::can_transceiver();
  hal::print(*console, "Waiting For CAN messages\n");
  while (true) {
    if (current_pos != can->receive_cursor()) {
      can_util::print_can_message(*console, can->receive_buffer()[current_pos]);
      current_pos = (current_pos + 1) % can->receive_buffer().size();
    }
  }
}
}  // namespace sjsu::drivers
