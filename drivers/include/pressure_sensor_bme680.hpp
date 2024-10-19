// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>

// enum format {
//   binary, decimal, hex
// };

namespace sjsu::drivers {
class bme680
{
public:
  /// @brief Possible modes
  enum mode : hal::byte
  {
    sleep = 0b00,
    forced = 0b01,
  };

  /// @brief Possible oversampling values
  enum oversampling : hal::byte
  {
    oversampling_1 = 0b001,
    oversampling_2 = 0b010,
    oversampling_4 = 0b011,
    oversampling_8 = 0b100,
    oversampling_16 = 0b0101,
  };

  /// @brief Possible coefficients for the on board iir filter
  enum filter_coeff : hal::byte
  {
    coeff_0 = 0b000,
    coeff_1 = 0b001,
    coeff_3 = 0b010,
    coeff_7 = 0b011,
    coeff_15 = 0b100,
    coeff_31 = 0b101,
    coeff_63 = 0b110,
    coeff_127 = 0b111,
  };

  /// @brief A list of named registers on the bme680
  enum registers : hal::byte
  {
    status = 0x73,
    reset = 0xE0,
    id = 0xD0,
    config = 0x75,
    ctrl_meas = 0x74,
    ctrl_hum = 0x72,
    ctrl_gas_1 = 0x71,
    ctrl_gas_0 = 0x70,
    // gas_wait_x = 0x70,
    // ctrl_gas_1 = 0x72,
    gas_r_lsb = 0x2B,
    gas_r_msb = 0x2A,
    humidity_lsb = 0x26,
    humidity_msb = 0x25,
    temp_xlsb = 0x24,
    temp_lsb = 0x23,
    temp_msb = 0x22,
    pressure_xlsb = 0x21,
    pressure_lsb = 0x20,
    pressure_msb = 0x1F,
    eas_status_0 = 0x1D,

  };

  /// The device address when 0x76 is connected to GND.
  static constexpr hal::byte address_ground = 0x76;

  /// @brief Reads the "Chip Ident" register
  /// @return Should return the address of the chip
  hal::byte read_addr();

  /// @brief Reset the sensor
  /// @return
  void soft_reset();

  /// @brief Set the coefficient value of the onboard iir filter.
  /// @param coeff
  /// @return v
  void set_filter_coefficient(filter_coeff coeff);
  /// @brief Set the oversampling values for each sensor.
  /// @param temp_osr Temperature oversampling
  /// @param press_osr Pressure oversampling
  /// @param humid_osr Humidity oversampling
  /// @return
  void set_oversampling(oversampling temp_osr,
                        oversampling press_osr,
                        oversampling humid_osr);

  /// @brief Set the current mode of the sensor
  /// @param p_mode
  /// @return
  void set_mode(mode p_mode);

  /// @brief Gets the calibration coefficients on the chip
  /// @return
  void get_calibration_coefficients();

  /// @brief Readings from the sensor
  struct readings_t
  {
    double temperature = 0;
    double pressure = 0;
    double humidity = 0;
  };

  /// @brief Sets the mode to forced and reads a single measurement.
  /// @return The readings from the sensor
  [[nodiscard]] readings_t get_data();

  /// @brief Print calibration coefficients. For debugging
  /// @param console Console to print to
  void print_calibration_coefficients(hal::serial& console);

  // /// @brief Print a register. For debugging
  // /// @param console Console to print to.
  // /// @param register_address Register to read
  // /// @param format Format to print register as.
  // void print_register(hal::serial& console, hal::byte register_address,
  // format format = format::hex);

  /// @brief Construct a bme680
  /// @param p_i2c bus
  /// @param p_address address
  explicit bme680(hal::i2c& p_i2c, hal::byte p_address);

private:
  /// @brief Write a single byte to a register
  /// @param register_address Address of register
  /// @param value Value to write
  /// @return
  void write_register(hal::byte register_address, hal::byte value);
  /// @brief Burst read bytes starting from a register. Reads towards higher
  /// memory.
  /// @param register_address Address of register to begin reading from.
  /// @param out Buffer to store read bytes.
  /// @return
  void read_registers(hal::byte register_address, std::span<hal::byte> out);

  /// @brief Structure with compensated and fine temp. Used internally. Check
  /// the datasheet.
  struct compensated_temp
  {
    double t_comp;
    double t_fine;
  };

  /// @brief Structure to hold coefficient values read during
  /// get_calibration_coefficients(). Check the datasheet.
  struct
  {
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;

    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;

    int16_t par_h1;
    int16_t par_h2;
    int16_t par_h3;
    int16_t par_h4;
    int16_t par_h5;
    int16_t par_h6;

    int16_t par_h7;

    int16_t par_g1;
    int16_t par_g2;
    int16_t par_g3;

  } coefficients;

  /// @brief Correct temperature according to datasheet and calibration
  /// coeffcients. Check the datasheet.
  /// @param adc_temp Raw temperature reading from sensor
  /// @return compensated and fine temperature.
  compensated_temp compensate_temp(double adc_temp);
  /// @brief Correct pressure according to datasheet, fine temperature and
  /// calibration coefficients. Check the datasheet.
  /// @param t_fine Fine temperature calculated during compensate_temp()
  /// @param adc_press Raw pressure reading from sensor.
  /// @return Compensated pressure
  double compensate_pressure(double t_fine, double adc_press);
  /// @brief Correct humidity according to datasheet, compensated temperature,
  /// and calibration coefficienst. Check the datasheet.
  /// @param temp_comp Compensated temperature calculated during
  /// compensate_temp()
  /// @param adc_humid Raw humidity reading from sensor.
  /// @return Compensated humidity.
  double compensate_humidity(double temp_comp, double adc_humid);

  /// @brief I2C bus the device is on
  hal::i2c& m_i2c;

  /// @brief I2C address of the bme680
  hal::byte m_address;
};

}  // namespace sjsu::drivers