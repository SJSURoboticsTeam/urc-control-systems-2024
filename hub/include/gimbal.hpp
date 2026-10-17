#pragma once

#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/gyroscope.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <mission_control_manager.hpp>

namespace sjsu::hub {
/**
 * @brief estimates the angle
 */
class imu_angle_estimator
{
public:
  struct settings
  {
    /**
     * @brief the proportion of the
     */
    float tau;
  };

  imu_angle_estimator(hal::v5::strong_ptr<hal::steady_clock> p_clock,
                      hal::v5::strong_ptr<hal::accelerometer> p_accel,
                      hal::v5::strong_ptr<hal::gyroscope> p_gyro,
                      settings p_settings = { .tau = 0.2f });

  /**
   * @brief returns the last estimated yaw about z of the servo as calculated
   * from the sensor readings
   */
  hal::degrees get_angle_aboutx();

  /**
   * @brief returns the last estimated rotation about y of the servo as
   * calculated from the sensor readings
   */
  hal::degrees get_angle_abouty();

  /**
   * @brief updates all angle estimates
   *
   * @note this uses the clock passed to it to compute the time that has passed
   * since it was last called for use in calculations.
   */
  void update();

private:
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::accelerometer> m_accel;
  hal::v5::strong_ptr<hal::gyroscope> m_gyro;
  settings m_settings;
  hal::degrees m_estimated_aboutx;
  hal::degrees m_estimated_abouty;
  hal::u64 m_last_uptime;
  float delta_time();
};

/**
 * @brief clamped_servo simply clamps angles between a given min and max
 * range before calling position on hal::actuator::rc_servo16
 */
class clamped_servo : public hal::servo
{
public:
  struct settings
  {
    hal::degrees min_angle;
    hal::degrees max_angle;
  };

  clamped_servo(hal::strong_ptr<hal::servo> p_servo,
                     settings p_settings);

private:
  hal::strong_ptr<hal::servo> m_servo;
  settings m_settings;
  void driver_position(hal::degrees p_position) override;
};

/**
 * @brief gimbal implements a simple gimbal stabilizer.
 */
class gimbal
{
public:
  gimbal(hal::v5::strong_ptr<hal::servo> p_servo);

  /**
   * @brief sets the target angle for the gimbal
   */
  void set_target(hal::degrees p_angle);

  /**
   * @brief updates the gimbal given a new estimate for current angle, should be
   * called periodically.
   */
  void update(hal::degrees p_angle_estimate);

private:
  hal::v5::strong_ptr<hal::servo> m_servo;
  hal::degrees m_target_angle;
};

class mast_gimbal_controller
{
public:
  struct components
  {
    hal::v5::strong_ptr<imu_angle_estimator> angle_estimator;
    hal::v5::strong_ptr<gimbal> mast_gimbal_abouty;
    clamped_servo mast_servo_aboutx;
    clamped_servo wheels_servo_aboutx;
  };

  // components are extracted into a struct because positional parameters with
  // similar types are easy to mix up
  mast_gimbal_controller(components p_comp);
  void set_target(gimbal_target_request const& target);
  void update();

private:
  components m_comp;
};
}  // namespace sjsu::hub
