#include <libhal/units.hpp>
#include <libhal/steady_clock.hpp>

constexpr hal::time_duration sec_to_hal_time_duration(float p_time)
{
  return static_cast<hal::time_duration>(static_cast<long long>(p_time * 1e9f));
}
constexpr float hal_time_duration_to_sec(hal::time_duration p_time)
{
  return static_cast<float>(p_time.count()) * 1e-9f;
}
constexpr hal::time_duration get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}