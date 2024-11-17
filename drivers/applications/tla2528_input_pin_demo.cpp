#include "../hardware_map.hpp"
#include <array>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/units.hpp>
#include <tla2528.hpp>
#include <tla2528_adapters.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  auto& terminal = *p_framework.terminal;
  auto& i2c = *p_framework.i2c;
  auto& steady_clock = *p_framework.steady_clock;
  tla2528 gpi_expander = tla2528(i2c);
  constexpr hal::input_pin::settings input_pin_config = { .resistor =
                                                  hal::pin_resistor::none };
  std::array<tla2528_input_pin, 8> gpis{
    make_input_pin(gpi_expander, 0, input_pin_config),
    make_input_pin(gpi_expander, 1, input_pin_config),
    make_input_pin(gpi_expander, 2, input_pin_config),
    make_input_pin(gpi_expander, 3, input_pin_config),
    make_input_pin(gpi_expander, 4, input_pin_config),
    make_input_pin(gpi_expander, 5, input_pin_config),
    make_input_pin(gpi_expander, 6, input_pin_config),
    make_input_pin(gpi_expander, 7, input_pin_config)
  };

  while (true) {
    hal::print(terminal, "\nvalues:");
    for (int i = 0; i < 8; i++) {
      hal::print<4>(terminal, "%x", gpis[i].level());
    }
    hal::delay(steady_clock, 500ms);
  }
}
}  // namespace sjsu::drivers