#include "../hardware_map.hpp"
#include "../include/tmag5273.hpp"
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  // configure drivers
  auto& i2c2 = *p_framework.i2c;
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;  
  tmag5273 tmag(i2c2, clock);

  hal::print<64>(terminal, "Tmag Instantiated!\n");
  tmag.defualt_config();
  hal::print<64>(terminal, "Default config sent!\n");

  while (true) {
    tmag5273::data rd = tmag.read();

    hal::print<64>(terminal, "Temperature: %f\n", rd.temperature);
    hal::print<64>(terminal, "X_field: %f\n", rd.x_field);
    hal::print<64>(terminal, "Y_field: %f\n", rd.y_field);
    hal::print<64>(terminal, "Z_field: %f\n", rd.z_field);
    hal::print<64>(terminal, "Angle: %f\n", rd.angle);
    hal::print<64>(terminal, "Magnitude: %f\n", rd.magnitude);
    hal::delay(clock, 500ms);
  }
}
}  // namespace sjsu::drivers