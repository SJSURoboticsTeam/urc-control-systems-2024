#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <adc1283.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
void application()
{

    auto console = resources::console();
    auto clock = resources::clock();

    hal::print(*console, "hello\n");
    hal::delay(*clock, 500ms);

    auto spi = resources::spi();
    auto cs = resources::spi_chip_select();

    hal::print(*console, "spi init ok\n");

    auto adc = adc1283(spi, cs, 3.3f);

    hal::print(*console, "adc init ok\n");

    while (true) {
        hal::print(*console, "ADC1283 Readings:\n");
        for (hal::byte channel = 0; channel < adc1283::channel_count; channel++){
            auto raw = adc.read_channel(channel);
            auto voltage = adc.adc_code_to_voltage(raw);
            hal::print<64>(*console,
                            "CH%d: %4u (%.4f V)\n",
                            channel,
                            raw,
                            voltage);
        }
        hal::print<64>(*console, "\n");
        hal::delay(*clock, 500ms);
    }
}
}   // namespace sjsu::drivers