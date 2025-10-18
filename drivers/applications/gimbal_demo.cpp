#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include <gimbal.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application()
{
  // configure drivers
  auto clock = resources::clock();
  auto terminal = resources::console();

  while (true) {
    hal::print(*terminal, "Hello World!\n");
    hal::delay(*clock, 100ms);
  }
}
}  // namespace sjsu::drivers