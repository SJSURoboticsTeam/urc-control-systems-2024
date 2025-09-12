#include "../hardware_map.hpp"
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto encoder = resources::encoder();

  while (true) {
    auto readings = encoder->read();
    hal::print<128>(*console, "%f\n", readings.angle);
    hal::delay(*clock, 200ms);
  }
}
}  // namespace sjsu::drivers