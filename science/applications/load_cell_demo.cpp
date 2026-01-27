#include <cinttypes>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

namespace  sjsu::science {
    void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto adc = resources::adc_0();

  hal::print(*console, "Load Cell Starting...\n");

  while (true) {
    using namespace std::chrono_literals;
    auto percent = adc->read();
    hal::print<128>(*console,
                    "LOAD CELL PERCENT: %" PRId32 "%\n", static_cast<std::int32_t>(percent * 100));
    hal::delay(*clock, 100ms);
  }
}

}