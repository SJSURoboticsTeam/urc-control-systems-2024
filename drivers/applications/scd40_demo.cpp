#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/scd40.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application()
{
  // configure drivers
  auto i2c = resources::i2c();
  auto clock = resources::clock();
  auto console = resources::console();

  auto scd40_sensor = scd40(i2c, clock);

  // hal::print<64>(terminal, "Hello World!");

  while (false) {
    // get settings test
    //  scd40_sensor.stop();
    //  auto get = scd40_sensor.get_settings();
    //  auto temp = get.temp_offset;
    //  auto alt = get.altitude;
    //  hal::print<64>(terminal, "%-5.2f\t%-5.2f\n", temp, alt);

    // periodic readings are only updated every 5000ms (temperature)
    hal::delay(*clock, 5000ms);
    auto rd = scd40_sensor.read();
    auto co2_levels = rd.co2;
    auto temp = rd.temp;
    auto RH_levels = rd.rh;

    // hal::print<64>(terminal, "%-5.2f\t%-5.2f\t%-5.2f\n", co2_levels, temp,
    // RH_levels); hal::delay(clock, 500ms);

    hal::print<64>(*console, "CO2 Levels: %f\n", co2_levels);
    hal::print<64>(*console, "Temperature %f\n", temp);
    hal::print<64>(*console, "RH Levels: %f\n", RH_levels);
  }
}
}  // namespace sjsu::drivers