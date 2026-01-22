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

  hal::print(*console, "ADC Application Starting...\n");

  while (true) {
    using namespace std::chrono_literals;
    auto percent = adc->read();
    // Get current uptime
    auto uptime = clock->uptime();
    hal::print<128>(*console,
                    "%" PRId32 "%%: %" PRIu32 "ns\n",
                    static_cast<std::int32_t>(percent * 100),
                    static_cast<std::uint32_t>(uptime));
    hal::delay(*clock, 100ms);
  }
}

}