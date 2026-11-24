#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
namespace sjsu::drivers::control_util {

class slew_rate_limiter
{
public:
  /**
   * @param p_clock clock used by slew rate limiter
   * @param p_slew_rate the slew rate in value/sec
   */
  slew_rate_limiter(hal::strong_ptr<hal::steady_clock> p_clock,
                    float p_slew_rate,
                    float p_starting_value = 0.0f);
  float update(float p_target);
  void set_slew_rate(float p_slew_rate);

private:
  hal::strong_ptr<hal::steady_clock> m_clock;
  float m_slew_rate;
  float m_last_update_time;
  float m_last_value;
};
}  // namespace sjsu::drivers::control_util
