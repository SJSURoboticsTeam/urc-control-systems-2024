#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <color_sensor_opt4048.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

void probe_bus(hal::v5::strong_ptr<hal::i2c> i2c,
               hal::v5::strong_ptr<hal::serial> console)
{
  hal::print(*console, "\n\nProbing i2c2\n");
  for (hal::byte addr = 0x08; addr < 0x78; addr++) {
    if (hal::probe(*i2c, addr)) {
      hal::print<8>(*console, "0x%02X  ", addr);
    } else {
      hal::print(*console, " --   ");
    }
    if (addr % 8 == 7) {
      hal::print(*console, "\n");
    }
  }
  hal::print(*console, "\n");
}

namespace sjsu::drivers {

void application()
{
  // configure drivers
  auto i2c2 = resources::i2c();
  auto clock = resources::clock();
  auto terminal = resources::console();

  hal::print(*terminal, "hi\n");
  probe_bus(i2c2, terminal);

  // create the stuff here
  auto color_sensor = drivers::opt4048(i2c2, clock, terminal);
  while (true) {
    hal::print(*terminal, "\nnew measurement\n");
    auto readings = color_sensor.get_data();
    hal::print<40>(
      *terminal, "R: %f\t G: %f\t B: %f\n", readings.r, readings.g, readings.b);
    hal::delay(*clock, 50ms);
  }
}
}  // namespace sjsu::drivers