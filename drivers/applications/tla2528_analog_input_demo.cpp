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
  tla2528 adc_mux = tla2528(i2c);
  std::array<tla2528_adc, 8> adcs{ make_adc(adc_mux, 0), make_adc(adc_mux, 1),
                                   make_adc(adc_mux, 2), make_adc(adc_mux, 3),
                                   make_adc(adc_mux, 4), make_adc(adc_mux, 5),
                                   make_adc(adc_mux, 6), make_adc(adc_mux, 7) };

  while (true) {
    hal::print(terminal, "\nvalues:\n");
    for (int i = 0; i < 8; i++) {
      hal::print<64>(terminal, "%d:%f\n", i, adcs[i].read());
    }
    hal::delay(steady_clock, 500ms);
  }
}
}  // namespace sjsu::drivers