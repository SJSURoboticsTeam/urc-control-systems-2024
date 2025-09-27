#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/apds9960.hpp"

namespace sjsu::drivers {
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto counter = resources::clock();
  auto terminal = resources::console();
  hal::print<64>(*terminal, "Clock and Console Created \n");
  auto i2c2 = resources::i2c();
  hal::print<64>(*terminal, "i2c Created \n");

  apds9960 adps9960_sensor = apds9960(i2c2, counter, terminal);
  hal::print<64>(*terminal, "Sensor Created \n");

  // not sure why it's taking a long time to print.. or to create sensor

  while (true) {
    hal::print<64>(*terminal, "Loop Time! \n");
    hal::delay(*counter, 50ms);

    apds9960::color read_color = adps9960_sensor.readColor();

    hal::print<64>(*terminal, "Red: %f\n", read_color.red_data);
    hal::print<64>(*terminal, "Green: %f\n", read_color.green_data);
    hal::print<64>(*terminal, "Blue: %f\n", read_color.blue_data);
    hal::print<64>(*terminal, "Clear: %f\n", read_color.clear_data);
  }
}
}  // namespace sjsu::drivers