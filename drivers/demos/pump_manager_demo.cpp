#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/pump_manager.hpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

hal::status application(application_framework& p_framework)
{
  // configure drivers
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;
  
  auto pump_manager = HAL_CHECK(pump_manager::create(
    clock,
    *p_framework.deionized_water_pump,
    *p_framework.sample_pump,
    *p_framework.molisch_reagent_pump,
    *p_framework.sulfuric_acid_pump,
    *p_framework.biuret_reagent
    ));

    while(true){
        hal::print<64>(terminal, "hello i am working");
        pump_manager.pump(pump_manager::pumps::DEIONIZED_WATER, 1000ms);
    }

  return hal::success();
}
}  // namespace sjsu::drivers