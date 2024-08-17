#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/pressure_sensor_bme680.hpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

void probe_bus(hal::i2c& i2c, hal::serial& console) {
  hal::print(console, "\n\nProbing i2c2\n");
  for(hal::byte addr = 0x08; addr < 0x78; addr++) {
    if (hal::probe(i2c, addr)) {
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


static int get_bit_value(hal::byte value, hal::byte bit_position) {
  return (value >> bit_position) & 0x01;
}

double calculate_height(double pressure) {
  double press_sea = 101325;
  double std_tmp = 23 + 273;
  double std_tmp_lapse = -0.0065; // K/m
  double atmos_layer_h = 0.0;
  double R = 8.31432; // Nm/(mol * K)
  double g = 9.80665; // m/s^2
  double M = 0.0289644; // kg/mol

  return atmos_layer_h + (std_tmp / std_tmp_lapse) * (pow(pressure / press_sea, -R * std_tmp_lapse / (g * M)) - 1);

}


void print_binary(hal::serial& console, hal::byte val) {
  hal::print<11>(console, "0b%c%c%c%c%c%c%c%c",
        get_bit_value(val, 7) + '0', // '1' if the bit is set, '0' if its not
        get_bit_value(val, 6) + '0',
        get_bit_value(val, 5) + '0',
        get_bit_value(val, 4) + '0',
        get_bit_value(val, 3) + '0',
        get_bit_value(val, 2) + '0',
        get_bit_value(val, 1) + '0',
        get_bit_value(val, 0) + '0');
}

namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  // configure drivers
  auto& i2c = *p_framework.i2c;
  auto& clock = *p_framework.steady_clock;
  auto& console = *p_framework.terminal;

  probe_bus(i2c, console);
  
  auto bme = hal::bme::bme680(i2c, 0x77);

  bme.set_filter_coefficient(hal::bme::bme680::coeff_3);
  bme.set_oversampling(hal::bme::bme680::oversampling_1, hal::bme::bme680::oversampling_2, hal::bme::bme680::oversampling_16);
  
  auto readings = bme.get_data();
  double average_pressure = 0.0;
  int samples = 100;
  for(int i = 0; i < samples; i ++) {
    readings = bme.get_data();
    hal::delay(clock, 10ms);
    average_pressure += readings.pressure;
  }
  average_pressure /= samples;
  
  average_pressure = 0.0;
  samples = 100;
  for(int i = 0; i < samples; i ++) {
    readings = bme.get_data();
    hal::delay(clock, 10ms);
    average_pressure += readings.pressure;
  }
  average_pressure /= samples;

  double init_height = calculate_height(average_pressure);

  print<40>(console, "init height: %fm\n", init_height);

  constexpr int samples_per_read_out = 20;
  int i = 0;
  while (true) {

    // led.level(true);
    readings = bme.get_data();
    double height = calculate_height(readings.pressure);

    // led.level(false);
    hal::delay(clock, 50ms);
    
    i++;
    if(i % samples_per_read_out == 0) {
      hal::print<40>(console, "| °C   | %-8s | %-4s |   m  |\n", "kPa", "%");
      hal::print<40>(console, "| %-3.1f | %-8.2f | %-4.1f | %-4.1f |\n", readings.temperature, average_pressure / 1000, readings.humidity, height - init_height);
    }
  }

}
}  // namespace sjsu::drivers