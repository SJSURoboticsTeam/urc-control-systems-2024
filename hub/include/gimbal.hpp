#pragma once
#include <algorithm>
#include <cmath>

#include <libhal-actuator/rc_servo.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include <sensor_sources.hpp>

namespace sjsu::hub {

struct gimbal_control_settings
{
  float tau = 0.2f;  // For the complementary filter
};

class gimbal
{
public:
  /**
   * @param p_x_servo servo motor for yaw (left/right)
   * @param p_y_servo servo motor for pitch (up/down), tilt compensated
   * @param p_min_angle minimum servo angle from rc_servo settings
   * @param p_max_angle maximum servo angle from rc_servo settings
   * @param p_pitch_min minimum pitch servo angle (default 30)
   * @param p_pitch_max maximum pitch servo angle (default 150)
   */
  gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_x_servo,
         hal::v5::strong_ptr<hal::actuator::rc_servo16> p_y_servo,
         hal::degrees p_min_angle,
         hal::degrees p_max_angle,
         hal::degrees p_pitch_min = 0.0f,
         hal::degrees p_pitch_max = 180.f);

  /**
   * @brief sets both yaw and pitch targets at once from a single CAN command
   *
   * @param p_x_deg target yaw angle (0-180, 0=left)
   * @param p_y_deg target pitch angle (clamped to pitch range)
   */
  void set_target(hal::degrees p_x_deg, hal::degrees p_y_deg);

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
  void update_y_servo(float p_delta_time,
                      sensor_axis const& p_accel,
                      sensor_axis const& p_gyro);

  /**
   * @brief returns the last commanded x servo angle
   * @return x angle as uint8_t (0-180)
   */
  uint8_t get_x_angle() const;

  /**
   * @brief returns the last commanded y servo angle
   * @return y angle as uint8_t (within pitch limits)
   */
  uint8_t get_y_angle() const;

private:
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_x_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_y_servo;

  // Full servo range (used for X/yaw)
  hal::degrees m_min_angle, m_max_angle;

  // Restricted pitch range (used for Y/pitch)
  hal::degrees m_pitch_min, m_pitch_max;

  hal::degrees m_curr_y_servo_angle, m_curr_x_servo_angle;

  // Complementary filter output
  float m_filtered_pitch;

  // Pitch offset from level, set by MC via set_pitch_target
  float m_tar_pitch;

  gimbal_control_settings m_settings;
};
}  // namespace sjsu::hub