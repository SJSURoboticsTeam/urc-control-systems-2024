#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../../platform-implementations/co2_sensor_sc40.cpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::science {

hal::status application(application_framework& p_framework)
{
  // configure drivers
  auto& i2c2 = *p_framework.i2c;
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;

  auto co2_sensor = HAL_CHECK(science::co2_sensor::create(i2c2, clock));

  while (true) {
    hal::delay(clock, 500ms);
    auto co2_level = HAL_CHECK(co2_sensor.read_co2());
    hal::print<64>(terminal, "C02: %d\n", co2_level);
    hal::delay(clock, 1ms);
  }

  return hal::success();
}
}  // namespace sjsu::science