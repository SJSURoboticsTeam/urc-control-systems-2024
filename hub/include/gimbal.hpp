#pragma once
#include <algorithm>
#include <cmath>

#include <libhal-actuator/rc_servo.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <libhal/gyroscope.hpp>
#include <libhal/accelerometer.hpp>

namespace sjsu::hub {

struct gimbal_control_settings
{
  float tau = 0.2f;  // For the complementary filter
};

class gimbal
{
public:
  /**
   * @param p_yaw_servo servo motor for yaw (left/right)
   * @param p_pitch_servo servo motor for pitch (up/down), tilt compensated
   * @param p_yaw_min minimum servo angle from rc_servo settings
   * @param p_yaw_max maximum servo angle from rc_servo settings
   * @param p_pitch_min minimum pitch servo angle (default 30)
   * @param p_pitch_max maximum pitch servo angle (default 150)
   */
  gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_yaw_servo,
         hal::v5::strong_ptr<hal::actuator::rc_servo16> p_pitch_servo,
         hal::degrees p_yaw_min,
         hal::degrees p_yaw_max,
         hal::degrees p_pitch_min = 0.0f,
         hal::degrees p_pitch_max = 180.f);

  /**
   * @brief sets both yaw and pitch targets at once
   *
   * @param p_yaw_deg target yaw angle (0-180, 0=left)
   * @param p_pitch_deg target pitch angle (clamped to pitch range)
   */
  void set_target(hal::degrees p_yaw_deg, hal::degrees p_pitch_deg);

  /**
   * @brief sets the yaw servo to a target angle directly
   *
   * @param p_yaw_deg target yaw angle, clamped to full servo range
   */
  void set_yaw_target(hal::degrees p_yaw_deg);

  /**
   * @brief sets the pitch target offset. Converted from external range
   * to internal offset from level.
   *
   * @param p_pitch_deg target pitch angle
   */
  void set_pitch_target(hal::degrees p_pitch_deg);

  /**
   * @brief compensates the pitch servo for rover tilt using IMU data.
   * Uses a complementary filter to estimate tilt, then directly
   * compensates the servo angle to keep the camera level.
   *
   * @param p_delta_time time since last update in seconds
   * @param p_accel accelerometer reading for tilt estimation
   * @param p_gyro gyroscope reading for complementary filter
   */
  void update_pitch_servo(float p_delta_time,
                          hal::accelerometer::read_t const& p_accel,
                          hal::gyroscope::read_t const& p_gyro);

  /**
   * @brief returns the last commanded x servo angle
   * @return x angle as (0-180)
   */
  hal::degrees get_yaw_angle() const;

  /**
   * @brief returns the last commanded y servo angle
   * @return y angle as (within pitch limits)
   */
  hal::degrees pitch() const;

private:
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_yaw_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_pitch_servo;

  // Full servo range (used for X/yaw)
  hal::degrees m_yaw_min, m_yaw_max;

  // Restricted pitch range (used for Y/pitch)
  hal::degrees m_pitch_min, m_pitch_max;

  hal::degrees m_curr_yaw_servo_angle, m_curr_pitch_servo_angle;

  // Complementary filter output
  float m_filtered_sensor_pitch = 0.0f;

  // Pitch offset from level, set via set_pitch_target
  float m_target_pitch = 0.0f;

  gimbal_control_settings m_settings;
};
}  // namespace sjsu::hub
