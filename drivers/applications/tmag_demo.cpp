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
  hal::print<64>(terminal, "Hello World!\n");

  // while (true) {
  //   using namespace std::literals;

  //   constexpr hal::byte first_i2c_address = 0x08;
  //   constexpr hal::byte last_i2c_address = 0x78;

  //   hal::print(console, "Devices Found: ");

  //   for (hal::byte address = first_i2c_address; address < last_i2c_address;
  //        address++) {
  //     // This can only fail if the device is not present
  //     if (hal::probe(i2c2, address)) {
  //       hal::print<12>(console, "0x%02X ", address);
  //     }
  //   }

  //   print(console, "\n");
  //   hal::delay(clock, 1s);
  // }
  
  tmag5273 tmag(i2c2, clock);

  hal::print<64>(terminal, "TMAG Intialized!");
  tmag.defualt_config();
  hal::print<64>(terminal, "TMAG Configured!");

  while (true) {
    tmag5273::data rd = tmag.read();

    hal::print<64>(terminal, "Temperature: %f\n", rd.temperature);
    hal::print<64>(terminal, "X_field: %f\n", rd.x_field);
    hal::print<64>(terminal, "Y_field: %f\n", rd.y_field);
    hal::print<64>(terminal, "Z_field: %f\n", rd.z_field);
    hal::print<64>(terminal, "Angle: %d\n", rd.angle);
    hal::print<64>(terminal, "Magnitude: %d\n", rd.magnitude);
    hal::delay(clock, 500ms);
  }
}
}  // namespace sjsu::drivers