#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
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

/**
 * @brief takes a value and places it in a bounds as if it were bounds of a ring
 * or clock. You can use this to convert a angle to the bounds of -pi to pi or
 * some some other similar.
 * @param p_value the value to place in bounds
 * @param p_lower the lower limit of the bounds to place the value in
 * @param p_upper the upper limit of the bounds to place the value in
 */
float modulus_range(float p_value, float p_lower, float p_upper);

class unrolled_rotation_sensor : public hal::rotation_sensor
{
public:
  unrolled_rotation_sensor(
    hal::v5::strong_ptr<hal::rotation_sensor> p_wraping_sensor,
    hal::degrees p_raw_reading_wrap_distance = 360.0f,
    hal::degrees p_starting_reading = 0.0f);

private:
  hal::v5::strong_ptr<hal::rotation_sensor> m_wraping_sensor;
  hal::degrees m_half_raw_reading_wrap_distance;
  hal::degrees m_last_reading;
  read_t driver_read() override;
};

}  // namespace sjsu::drivers::control_util
