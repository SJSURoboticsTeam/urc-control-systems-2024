#include <cmath>
#include <libhal/adc.hpp>
#include <libhal/input_pin.hpp>

#include "../include/mq4.hpp"

namespace sjsu::drivers {

mq4::mq4(hal::adc& p_adc)
  : m_adc(p_adc) {};

float mq4::get_parsed_data()
{
  float raw_adc_value = read_raw_adc();

  float ppm_value = static_cast<float>(std::pow(1000 * raw_adc_value, -2.95));
  return ppm_value;
}

float mq4::read_raw_adc()
{
  float raw_ratio_average = 0;
  int read_count = 10;
  for (int i = 0; i < read_count; i++) {
    raw_ratio_average += m_adc.read();
  }

  raw_ratio_average /= read_count;

  return raw_ratio_average;
}
};  // namespace sjsu::drivers