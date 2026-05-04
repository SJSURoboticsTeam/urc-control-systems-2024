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
  auto spi = resources::spi();
  auto chip_select = resources::spi_chip_select();

  adc1283 adc_driver = adc1283(spi, chip_select, hal::volts(3.3f));
  std::array<adc1283_adc, 8> adcs{ make_adc(adc_driver, 0),
                                   make_adc(adc_driver, 1),
                                   make_adc(adc_driver, 2),
                                   make_adc(adc_driver, 3),
                                   make_adc(adc_driver, 4),
                                   make_adc(adc_driver, 5),
                                   make_adc(adc_driver, 6),
                                   make_adc(adc_driver, 7) };

  while (true) {
    hal::print(*console, "ADC1283 Readings:\n");
    for (int i = 0; i < 8; i++) {
      hal::print<64>(*console, "CH%d: %.4f\n", i, adcs[i].read());
    }
    hal::print<64>(*console, "\n");
    hal::delay(*clock, 500ms);
  }
}
}  // namespace sjsu::drivers