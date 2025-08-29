#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/pressure_sensor_bme680.hpp"
#include "../include/soil_sensor_sht21.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{
  // configure drivers
  // auto& i2c = *p_framework.i2c;
  auto i2c = resources::i2c();
  auto clock = resources::clock();
  auto console = resources::console();

  sht21 m_sht21 = sht21(i2c);

  while (true) {
    hal::print<32>(*console, "Temp: %f\n", m_sht21.get_temperature());
    hal::print<32>(*console, "Humid: %f\n", m_sht21.get_relative_humidity());
    hal::delay(*clock, 50ms);
  }
}
}  // namespace sjsu::drivers