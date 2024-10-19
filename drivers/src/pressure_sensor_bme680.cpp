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

#include "../include/pressure_sensor_bme680.hpp"
#include <libhal/units.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

bme680::bme680(hal::i2c& p_i2c, hal::byte p_address)
  : m_i2c(p_i2c)
{
  m_address = p_address;
  read_addr();
  soft_reset();
  get_calibration_coefficients();
}

void bme680::soft_reset()
{
  write_register(0xE0, 0xB6);
}

// a nodiscard attribute was removed when updating to libHAL 3.0
hal::byte bme680::read_addr()
{
  std::array<hal::byte, 1> out;
  read_registers(registers::id, out);
  return out[0];
}

void bme680::set_filter_coefficient(filter_coeff coeff)
{
  std::array<hal::byte, 1> out;
  read_registers(registers::config, out);

  hal::byte original_value = out[0];
  hal::byte new_value = (original_value & (0b11100011)) | (coeff << 2);

  write_register(registers::config, new_value);
}

void bme680::set_oversampling(oversampling temp_osr,
                              oversampling press_osr,
                              oversampling humid_osr)
{
  std::array<hal::byte, 2> out;
  read_registers(registers::ctrl_hum, out);

  hal::byte ctrl_meas_original_value = out[1];
  hal::byte ctrl_meas_new_value = (ctrl_meas_original_value & (0b00000011)) |
                                  (((hal::byte)temp_osr) << 5) |
                                  (((hal::byte)press_osr) << 2);

  hal::byte ctrl_hum_original_value = out[0];
  hal::byte ctrl_hum_new_value =
    (ctrl_hum_original_value & (0b11111000)) | (((hal::byte)humid_osr) << 0);

  write_register(registers::ctrl_hum, ctrl_hum_new_value);
  // hal::delay(*m_steady_clock, 20ms); // Wait 10ms for readings.s
  write_register(registers::ctrl_meas, ctrl_meas_new_value);
}

void bme680::set_mode(mode p_mode)
{
  std::array<hal::byte, 1> out;
  read_registers(registers::ctrl_meas, out);

  hal::byte original_value = out[0];
  hal::byte new_value = (original_value & (0b11111100)) | (p_mode << 0);

  write_register(registers::ctrl_meas, new_value);
}

static int16_t combine(uint8_t upper, uint8_t lower)
{
  return (((int16_t)upper) << 8) | ((int16_t)lower);
}

void bme680::get_calibration_coefficients()
{
  // CHECK THE DATASHEET
  std::array<hal::byte, 0xA0 - 0x8A + 1> reg8A_regA0;
  read_registers(0x8A, reg8A_regA0);

  auto span_reg8A_regA0 = std::span(reg8A_regA0);
  read_registers(0x8A, span_reg8A_regA0.subspan(0, 8));

  std::array<hal::byte, 0xEE - 0xE1 + 1> regE1_regEE;
  read_registers(0xE1, regE1_regEE);

  coefficients.par_t1 =
    combine(regE1_regEE[0xEA - 0xE1], regE1_regEE[0xE9 - 0xE1]);
  coefficients.par_t2 =
    combine(reg8A_regA0[0x8B - 0x8A], reg8A_regA0[0x8A - 0x8A]);
  coefficients.par_t3 = combine(0, reg8A_regA0[0x8C - 0x8A]);

  coefficients.par_p1 =
    combine(reg8A_regA0[0x8F - 0x8A], reg8A_regA0[0x8E - 0x8A]);
  coefficients.par_p2 =
    combine(reg8A_regA0[0x91 - 0x8A], reg8A_regA0[0x90 - 0x8A]);
  coefficients.par_p3 = combine(0, reg8A_regA0[0x90 - 0x8A]);
  coefficients.par_p4 =
    combine(reg8A_regA0[0x95 - 0x8A], reg8A_regA0[0x94 - 0x8A]);
  coefficients.par_p5 =
    combine(reg8A_regA0[0x97 - 0x8A], reg8A_regA0[0x96 - 0x8A]);
  coefficients.par_p6 = combine(0, reg8A_regA0[0x99 - 0x8A]);
  coefficients.par_p7 = combine(0, reg8A_regA0[0x98 - 0x8A]);
  coefficients.par_p8 =
    combine(reg8A_regA0[reg8A_regA0[0x9D - 0x8A]], reg8A_regA0[0x9C - 0x8A]);
  coefficients.par_p9 =
    combine(reg8A_regA0[0x9F - 0x8A], reg8A_regA0[0x9E - 0x8A]);
  coefficients.par_p10 = combine(0, reg8A_regA0[0xA0 - 0x8A]);

  coefficients.par_h1 = (((int16_t)regE1_regEE[0xE3 - 0xE1]) << 4) |
                        (regE1_regEE[0xE2 - 0xE1] & 0x0F);
  coefficients.par_h2 = (((int16_t)regE1_regEE[0xE1 - 0xE1]) << 4) |
                        (regE1_regEE[0xE2 - 0xE1] >> 4);
  coefficients.par_h3 = combine(0, regE1_regEE[0xE4 - 0xE1]);
  coefficients.par_h4 = combine(0, regE1_regEE[0xE5 - 0xE1]);
  coefficients.par_h5 = combine(0, regE1_regEE[0xE6 - 0xE1]);
  coefficients.par_h6 = combine(0, regE1_regEE[0xE7 - 0xE1]);
  coefficients.par_h7 = combine(0, regE1_regEE[0xE8 - 0xE1]);

  coefficients.par_g1 = combine(0, regE1_regEE[0xED - 0xE1]);
  coefficients.par_g2 =
    combine(regE1_regEE[0xEC - 0xE1], regE1_regEE[0xEB - 0xE1]);
  coefficients.par_g3 = combine(0, regE1_regEE[0xEE - 0xE1]);
}

bme680::readings_t bme680::get_data()
{
  // Set the bme680 into forced mode
  set_mode(mode::forced);
  // hal::delay(*m_steady_clock, 1ms); // Wait 1ms for readings.

  std::array<hal::byte, 8> buffer;
  read_registers(registers::pressure_msb, buffer);

  uint32_t humidity_raw = (((uint32_t)buffer[6]) << 8) | ((uint32_t)buffer[7]);
  uint32_t temperature_raw = (((uint32_t)buffer[3]) << 12) |
                             (((uint32_t)buffer[4]) << 4) |
                             ((uint32_t)buffer[5] >> 4);
  uint32_t pressure_raw = (((uint32_t)buffer[0]) << 12) |
                          (((uint32_t)buffer[1]) << 4) |
                          ((uint32_t)buffer[2] >> 4);

  compensated_temp temp = compensate_temp(temperature_raw);
  double pressure = compensate_pressure(temp.t_fine, pressure_raw);
  double humidity = compensate_humidity(temp.t_comp, humidity_raw);

  readings_t out;
  out.temperature = temp.t_comp;
  out.pressure = pressure;
  out.humidity = humidity;
  return out;
}

void bme680::write_register(hal::byte register_address, hal::byte value)
{
  hal::write(m_i2c,
             m_address,
             std::array<hal::byte, 2>{ register_address, value },
             hal::never_timeout());
}

void bme680::read_registers(hal::byte register_address,
                            std::span<hal::byte> out)
{
  hal::write_then_read(m_i2c,
                       m_address,
                       std::array<hal::byte, 1>{ register_address },
                       out,
                       hal::never_timeout());
}

bme680::compensated_temp bme680::compensate_temp(double temp_adc)
{
  // CHECK THE DATASHEET
  double var1 = ((((double)temp_adc) / 16384.0) -
                 (((double)coefficients.par_t1) / 1024.0)) *
                ((double)coefficients.par_t2);
  double var2 = (((((double)temp_adc) / 131072.0) -
                  (((double)coefficients.par_t1) / 8192.0)) *
                 ((((double)temp_adc) / 131072.0) -
                  (((double)coefficients.par_t1) / 8192.0))) *
                (((double)coefficients.par_t3) * 16.0);
  compensated_temp out;
  out.t_fine = var1 + var2;
  out.t_comp = out.t_fine / 5120.0;
  return out;
}

double bme680::compensate_pressure(double t_fine, double press_adc)
{
  // CHECK THE DATASHEET

  double var1 = ((double)t_fine / 2.0) - 64000.0;
  double var2 = var1 * var1 * ((double)coefficients.par_p6 / 131072.0);
  var2 = var2 + (var1 * (double)coefficients.par_p5 * 2.0);
  var2 = (var2 / 4.0) + ((double)coefficients.par_p4 * 65536.0);
  var1 = ((((double)coefficients.par_p3 * var1 * var1) / 16384.0) +
          ((double)coefficients.par_p2 * var1)) /
         524288.0;
  var1 = (1.0 + (var1 / 32768.0)) * (double)coefficients.par_p1;
  double press_comp = 1048576.0 - (double)press_adc;
  press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
  var1 = ((double)coefficients.par_p9 * press_comp * press_comp) / 2147483648.0;
  var2 = press_comp * ((double)coefficients.par_p8 / 32768.0);
  double var3 = (press_comp / 256.0) * (press_comp / 256.0) *
                (press_comp / 256.0) * (coefficients.par_p10 / 131072.0);
  press_comp =
    press_comp +
    (var1 + var2 + var3 + ((double)coefficients.par_p7 * 128.0)) / 16.0;
  return press_comp;
}
double bme680::compensate_humidity(double temp_comp, double hum_adc)
{
  // CHECK THE DATASHEET

  double var1 = hum_adc - (((double)coefficients.par_h1 * 16.0) +
                           (((double)coefficients.par_h3 / 2.0) * temp_comp));
  double var2 =
    var1 *
    (((double)coefficients.par_h2 / 262144.0) *
     (1.0 + (((double)coefficients.par_h4 / 16384.0) * temp_comp) +
      (((double)coefficients.par_h5 / 1048576.0) * temp_comp * temp_comp)));
  double var3 = (double)coefficients.par_h6 / 16384.0;
  double var4 = (double)coefficients.par_h7 / 2097152.0;
  return var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
}

void bme680::print_calibration_coefficients(hal::serial& console)
{
  hal::print<512>(console,
                  "Temp Coefficients: %10d %10d %10d\n",
                  coefficients.par_t1,
                  coefficients.par_t2,
                  coefficients.par_t3);
  hal::print<512>(console,
                  "Press Coefficients:\n%10d %10d %10d\n%10d %10d %10d\n%10d "
                  "%10d %10d\n%10d %10d %10d\n%10d\n",
                  coefficients.par_p1,
                  coefficients.par_p2,
                  coefficients.par_p3,
                  coefficients.par_p4,
                  coefficients.par_p5,
                  coefficients.par_p6,
                  coefficients.par_p7,
                  coefficients.par_p8,
                  coefficients.par_p9,
                  coefficients.par_p10);
  hal::print<512>(
    console,
    "Humidity Coefficients:\n%10d %10d %10d\n%10d %10d %10d\n%10d\n",
    coefficients.par_h1,
    coefficients.par_h2,
    coefficients.par_h3,
    coefficients.par_h4,
    coefficients.par_h5,
    coefficients.par_h6,
    coefficients.par_h7);
}

};  // namespace sjsu::drivers
