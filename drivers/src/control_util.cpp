#include <algorithm>
#include <cmath>
#include <control_util.hpp>
#include <libhal/error.hpp>
#include <libhal/rotation_sensor.hpp>
#include <limits>
#include <time_util.hpp>

namespace sjsu::drivers::control_util {
slew_rate_limiter::slew_rate_limiter(hal::strong_ptr<hal::steady_clock> p_clock,
                                     float p_slew_rate,
                                     float p_starting_value)
  : m_clock(p_clock)
  , m_slew_rate(p_slew_rate)
  , m_last_value(p_starting_value)
{
  if (std::isnan(p_slew_rate) || p_slew_rate < 0) {
    throw hal::argument_out_of_domain(this);
  }
  m_last_update_time = drivers::time_util::get_clock_time_sec(*m_clock);
};

float slew_rate_limiter::update(float p_target)
{
  float cur_time = time_util::get_clock_time_sec(*m_clock);
  if (m_slew_rate == std::numeric_limits<float>::infinity()) {
    m_last_value = p_target;
  } else {
    float max_diff = std::abs(cur_time - m_last_update_time) * m_slew_rate;
    m_last_value =
      std::clamp(p_target, m_last_value - max_diff, m_last_value + max_diff);
  }
  m_last_update_time = cur_time;
  return m_last_value;
}

void slew_rate_limiter::set_slew_rate(float p_slew_rate)
{
  if (std::isnan(p_slew_rate) || p_slew_rate < 0) {
    throw hal::argument_out_of_domain(this);
  }
  m_slew_rate = p_slew_rate;
}

float modulus_range(float p_value, float p_lower, float p_upper)
{
  float diff = p_upper - p_lower;
  float offset = p_value - p_lower;
  // if less then push up
  if (offset < 0) {
    int rounds_below = -offset / diff + 1;
    offset += diff * rounds_below;
  }
  // if more
  else if (offset >= diff) {
    int rounds_above = offset / diff;
    offset -= diff * rounds_above;
  }
  return offset + p_lower;
}

float find_closest_wrapping(float p_value,
                            float p_wrap_distance,
                            float p_previous_value)
{
  if (not(p_wrap_distance > 0) ||
      p_wrap_distance == std::numeric_limits<float>::infinity()) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return modulus_range(p_value,
                       p_previous_value - p_wrap_distance / 2.0f,
                       p_previous_value + p_wrap_distance / 2.0f);
}

unrolled_rotation_sensor::unrolled_rotation_sensor(
  hal::v5::strong_ptr<hal::rotation_sensor> p_wraping_sensor,
  hal::degrees p_raw_reading_wrap_distance,
  hal::degrees p_starting_reading)
  : m_wrapping_sensor(p_wraping_sensor)
  , m_raw_reading_wrap_distance(p_raw_reading_wrap_distance / 2.0f)
  , m_last_reading(p_starting_reading)
{
  driver_read();
}

void unrolled_rotation_sensor::set_last_reading(hal::degrees p_reading)
{
  m_last_reading = p_reading;
}

hal::rotation_sensor::read_t unrolled_rotation_sensor::driver_read()
{
  m_last_reading = find_closest_wrapping(m_wrapping_sensor->read().angle,
                                         m_raw_reading_wrap_distance,
                                         m_last_reading);
  return { .angle = m_last_reading };
}

}  // namespace sjsu::drivers::control_util
