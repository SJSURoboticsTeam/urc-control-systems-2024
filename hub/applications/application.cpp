#include "../hardware_map.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

namespace sjsu::hub {

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "Received new hal::can_message { \n"
                 "    id: 0x%lX,\n"
                 "    length: %u \n"
                 "    payload = [ ",
                 p_message.id,
                 p_message.length);

  for (auto const& byte : p_message.payload) {
    hal::print<8>(p_console, "0x%02X, ", byte);
  }

  hal::print(p_console, "]\n}\n");
}
void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  auto console = resources::console();

  static constexpr auto baudrate = 100.0_kHz;

  can_bus_manager->baud_rate(baudrate);
  can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });
  constexpr auto allowed_id = 0x300;
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*can_transceiver, 0x300);

  while (true) {
    
    for (auto m = message_finder.find(); m.has_value();
         m = message_finder.find()) {
      print_can_message(*console, *m);
      auto& msg = *m;
        //if(m.hasvalue) reset watchdog timer
      if (msg.length == 2) {
        uint8_t position = msg.payload[0];
        //uint8_t offset = msg.payload[1];
        switch (position) {
          case 0x00:
            // move left
            break;
          case 0x01:
            // move right
            break;
          case 0x02:
            // move up
            break;
          case 0x03:
            // move down
            break;
        }
    }
  }
}
}
}  // namespace sjsu::hub