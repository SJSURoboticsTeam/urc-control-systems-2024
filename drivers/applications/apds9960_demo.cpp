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
  using namespace std::literals;

  // [[maybe_unused]] constexpr hal::byte first_i2c_address = 0x08;
  // [[maybe_unused]] constexpr hal::byte last_i2c_address = 0x78;
  // hal::print(*terminal, "I2C devices found: ");
  // for (hal::byte address = first_i2c_address; address < last_i2c_address;
  //      address++) {
  //   // this can only fail if the device is not present
  //   if (hal::probe(*i2c2, address)) {
  //     hal::print<12>(*terminal, "0x%02X ", address);
  //   }
  // }

  apds9960 adps9960_sensor = apds9960(i2c2, counter, terminal);

  hal::delay(*counter, 1000ms);

  hal::print<64>(*terminal, "Sensor Created \n");
  hal::delay(*counter, 150ms);

  hal::print<64>(*terminal, "Loop Time! \n");
  while (true) {
    hal::delay(*counter, 500ms);

    apds9960::color read_color = adps9960_sensor.readColor();

    // all values are 0 when printed w %f, values change when printed w %u

    // Raw Values
    hal::print<64>(*terminal,
                   "R:%u  G:%u  B:%u  C:%u\n",
                   read_color.red_data,
                   read_color.green_data,
                   read_color.blue_data,
                   read_color.clear_data);
  }
}
}  // namespace sjsu::drivers