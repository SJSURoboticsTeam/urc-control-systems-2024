#include <array>
#include <cstdio>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>
#include <resource_list.hpp>
#include <string_view>

using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{

  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();
  auto display = nhd0420d3z(*i2c);

  hal::delay(*clock, 1ms);
  display.power(true);

  constexpr std::string_view demoPrintFMessage = "d[%d]\nx[%x]\nf[%f]";
  constexpr int buffer_size = 256;
  std::array<hal::byte, buffer_size> printMessage;

  // Demo Printf capabilities
  std::snprintf(reinterpret_cast<char*>(printMessage.begin()),
                printMessage.size(),
                demoPrintFMessage.begin(),
                31,
                31,
                31.31f);

  display.display_message(demoPrintFMessage);
  hal::print(*console,
             std::string_view(reinterpret_cast<char*>(printMessage.begin()),
                              printMessage.size()));

  // String Input From Serial
  while (true) {
    printMessage = hal::read<buffer_size>(*console, hal::never_timeout());
    display.display_message(std::string_view(
      reinterpret_cast<char*>(printMessage.begin()), printMessage.size()));
    hal::print(*console, printMessage);
  }
}
}  // namespace sjsu::drivers