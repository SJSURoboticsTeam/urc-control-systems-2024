#include <array>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <adc1283.hpp>
#include <adc1283_adapters.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application()
{
  auto console = resources::console();
  auto clock = resources::clock();
  auto led = resources::status_led();

  while (true) {
    led->level(true);
    hal::print<64>(*console, "On\n");
    hal::delay(*clock, 1000ms);

    led->level(false);
    hal::print<64>(*console, "Off\n");
    hal::delay(*clock, 1000ms);
  }
}
}  // namespace sjsu::drivers