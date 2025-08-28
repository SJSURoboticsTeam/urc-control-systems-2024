#pragma once
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

// TODO: maybe later add CRC and config for altitude, ambient pressure,
// temperature offset (or look at setting persistent values)

namespace sjsu::drivers {
/**
 * @brief scd40 is a CO2, temperature, and Relative Humidity sensor
 */
class scd40
{
private:
  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::byte generate_crc(std::array<hal::byte, 2> data);
  bool validate_crc(std::array<hal::byte, 3> data);

  // TODO: revise output type
  enum addresses
  {  // deal with hal::byte later
    device_address = 0x62,
    start_periodic_measurement_first_half = 0x21,
    start_periodic_measurement_second_half = 0xb1,
    read_measurement_first_half = 0xec,
    read_measurement_second_half = 0x05,
    stop_periodic_measurement_first_half = 0x3f,
    stop_periodic_measurement_second_half = 0x86,
    set_temperature_offset_first_half = 0x24,
    set_temperature_offset_second_half = 0x1d,
    get_temperature_offset_first_half = 0x23,
    get_temperature_offset_second_half = 0x18,
    set_sensor_altitude_first_half = 0x24,
    set_sensor_altitude_second_half = 0x27,
    get_sensor_altitude_first_half = 0x23,
    get_sensor_altitude_second_half = 0x22,
    set_ambient_pressure_first_half = 0xe0,
    set_ambient_pressure_second_half = 0x00
  };

public:
  struct scd40_read_data
  {
    double co2;
    double temp;
    double rh;
  };

  struct settings
  {
    float set_temp = 4;
    float set_alt = 0;
    float set_pressure = -1;
  };

  struct scd40_settings
  {
    float temp_offset;
    float altitude;
  };
  scd40(hal::v5::strong_ptr<hal::i2c> p_i2c,
        hal::v5::strong_ptr<hal::steady_clock> p_clock);
  void start();
  scd40_read_data read();
  void stop();
  scd40_settings get_settings();
  void set_settings(settings);
};

}  // namespace sjsu::drivers