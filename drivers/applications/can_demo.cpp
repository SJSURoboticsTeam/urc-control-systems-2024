#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>


namespace sjsu::drivers {
void application(application_framework& p_framework)
{
  auto& console = *p_framework.terminal;
  auto& can = *p_framework.can;
  hal::print(console, "waiting for incoming messages");

  auto receive_handler = [&console](hal::can::message_t const& p_message) {
    hal::print<1024>(console,
                     "Received Message from ID: 0x%lX, length: %u \n"
                     "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                     "0x%02X, 0x%02X, 0x%02X ]\n",
                     p_message.id,
                     p_message.length,
                     p_message.payload[0],
                     p_message.payload[1],
                     p_message.payload[2],
                     p_message.payload[3],
                     p_message.payload[4],
                     p_message.payload[5],
                     p_message.payload[6],
                     p_message.payload[7]);
  };
  can.on_receive(receive_handler);
  while (true) {
    continue;
  }
}
}  // namespace sjsu::drivers