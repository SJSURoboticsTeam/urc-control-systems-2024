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
   * @param p_starting_value the starting output value
   */
  slew_rate_limiter(hal::strong_ptr<hal::steady_clock> p_clock,
                    float p_slew_rate,
                    float p_starting_value = 0.0f);
  /**
   * @brief calculates the new output signal given the new input signal. this
   * function is expected to be called periodically.
   *
   * @param p_target the target value
   * @returns slew rate limited value
   *
   * This function is expected to be called periodically. It a calculates a
   * allowed delta from previous output to new output based on time from
   * previous call. If update is not called for a long time the allowed delta
   * will be very large.
   *
   */
  float update(float p_target);
  /**
   * @param p_slew_rate the new slew rate to be used by future calculations
   */
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

/**
 * @brief finds closest value to p_previous_values that has the same modulo
 * p_wrap_distance as p_value
 * @param p_value the value to place in bounds
 * @param p_wrap_distance distance before reading wraps
 * @param p_previous_value value
 */
float find_closest_wrapping(float p_value,
                            float p_wrap_distance,
                            float p_previous_value = 0.0f);

/**
 * @brief takes a rotation sensor that will wrap around in terms of rotations
 *
 * it dose this by assuming that the encoder will travel less than half the
 * wraping distance between readings. It returns the closest position to the
 * previous reading that also wraps to current raw reading. To ensure accuracy
 * the driver should be read at a frequency that the assumption holds.
 */
class unrolled_rotation_sensor : public hal::rotation_sensor
{
public:
  /**
   * @param p_wraping_sensor the sensor being read from that wraps around
   * @param p_raw_reading_wrap_distance the distance traveled by the sensor
   * before wraping around to the same value once
   * @param p_starting_position the assumed starting position, on construction
   * the reading will be set to the closest value that wraps to the same raw
   * reading
   */
  unrolled_rotation_sensor(
    hal::v5::strong_ptr<hal::rotation_sensor> p_wraping_sensor,
    hal::degrees p_raw_reading_wrap_distance = 360.0f,
    hal::degrees p_starting_reading = 0.0f);
  /**
   * @param p_reading the actual postion or position that is withing half of
   * wraping ranging range of the actual position
   */
  void set_last_reading(hal::degrees p_reading);

private:
  hal::v5::strong_ptr<hal::rotation_sensor> m_wraping_sensor;
  hal::degrees m_raw_reading_wrap_distance;
  hal::degrees m_last_reading;
  read_t driver_read() override;
};

}  // namespace sjsu::drivers::control_util
