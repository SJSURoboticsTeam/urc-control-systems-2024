#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <libhal-lpc40/i2c.hpp>

#include "../include/pressure_sensor_bme680.hpp"
#include "../include/soil_sensor_sht21.hpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;



 hal::i2c::transaction_t probe(
  hal::i2c& p_i2c,
  hal::byte p_address, hal::steady_clock& p_steady_clock, hal::time_duration p_duration) {
  // p_data_in: empty placeholder for transcation's data_in
  std::array<hal::byte, 1> data_in;

  // p_timeout: no timeout placeholder for transaction's p_timeout
  hal::timeout auto timeout = hal::create_timeout(p_steady_clock, p_duration);

  return p_i2c.transaction(p_address, std::span<hal::byte>{}, data_in, timeout);
}

void probe_bus(hal::i2c& i2c, hal::serial& console, hal::steady_clock& p_steady_clock) {
  hal::print(console, "\n\nProbing i2c2\n");
  i2c = i2c;
  for(hal::byte addr = 0x08; addr < 0x78; addr++) {
    if (probe(i2c, addr, p_steady_clock, 10ms)) {
    // if(false){
      hal::print<8>(console, "0x%02X  ", addr);
    }else{
      hal::print(console, " --   ");
    }
    if(addr % 8 == 7) {
      hal::print(console, "\n");
    }
  }
  hal::print(console, "\n");

}


// static int get_bit_value(hal::byte value, hal::byte bit_position) {
//   return (value >> bit_position) & 0x01;
// }

// static void print_binary(hal::serial& console, hal::byte val) {
//   hal::print<11>(console, "0b%c%c%c%c%c%c%c%c",
//         get_bit_value(val, 7) + '0', // '1' if the bit is set, '0' if its not
//         get_bit_value(val, 6) + '0',
//         get_bit_value(val, 5) + '0',
//         get_bit_value(val, 4) + '0',
//         get_bit_value(val, 3) + '0',
//         get_bit_value(val, 2) + '0',
//         get_bit_value(val, 1) + '0',
//         get_bit_value(val, 0) + '0');
// }

namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  // configure drivers
  // auto& i2c = *p_framework.i2c;
  auto& clock = *p_framework.steady_clock;
  auto& console = *p_framework.terminal;

  auto i2c = hal::lpc40::i2c::get(2);

  hal::print(console, "reset?\n");


  probe_bus(i2c, console, clock);

  hal::sht::sht21 maybe_soil = hal::sht::sht21(i2c);
  while(maybe_soil.has_error()) {
    hal::print(console, "sensor create error :(\n");
    maybe_soil = hal::sht::sht21(i2c);
    hal::delay(clock, 50ms);
  }

  hal::sht::sht21 soil = maybe_soil.value();

//   int i = 0;
  while (true) {
    double maybe_temp = soil.get_temperature();
    double maybe_humid = soil.get_relative_humidity();
    if(maybe_temp.has_value()) {
      hal::print<32>(console, "Temp: %f\n", maybe_temp.value());
    }
    if(maybe_humid.has_value()) {
      hal::print<32>(console, "humid: %f\n", maybe_humid.value());
    }
    hal::delay(clock, 50ms);
  }

}
}  // namespace sjsu::drivers