#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../../platform-implementations/scd40.hpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::science {

void application(application_framework& p_framework)
{
  // configure drivers
  auto& i2c2 = *p_framework.i2c;
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;
  
  auto scd40_sensor = HAL_CHECK(scd40::create(i2c2, clock));

  while(true){
        //get settings test
        // scd40_sensor.stop();
        // auto get = HAL_CHECK(scd40_sensor.get_settings());
        // auto temp = get.temp_offset;
        // auto alt = get.altitude;
        // hal::print<64>(terminal, "%-5.2f\t%-5.2f\n", temp, alt);
  
        // periodic readings are only updated every 5000ms (temperature)
        hal::delay(clock, 5000ms);
        auto rd = HAL_CHECK(scd40_sensor.read());
        auto co2_levels = rd.co2;
        auto temp = rd.temp;
        auto RH_levels = rd.rh;


        // hal::print<64>(terminal, "%-5.2f\t%-5.2f\t%-5.2f\n", co2_levels, temp, RH_levels);
        // hal::delay(clock, 500ms);

        hal::print<64>(terminal, "CO2 Levels: %f\n", co2_levels);
        hal::print<64>(terminal, "Temperature %f\n", temp);
        hal::print<64>(terminal, "RH Levels: %f\n", RH_levels);

    }
}
}  // namespace sjsu::science