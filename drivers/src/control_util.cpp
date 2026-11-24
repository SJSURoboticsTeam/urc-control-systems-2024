#include <algorithm>
#include <cmath>
#include <control_util.hpp>
#include <libhal/error.hpp>
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
  m_slew_rate = p_slew_rate;
}

}  // namespace sjsu::drivers::control_util
