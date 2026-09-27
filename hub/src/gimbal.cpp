#include <gimbal.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/units.hpp>
#include <numbers>
namespace sjsu::hub {

gimbal::gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_yaw_servo,
               hal::v5::strong_ptr<hal::actuator::rc_servo16> p_pitch_servo,
               hal::degrees p_yaw_min,
               hal::degrees p_yaw_max,
               hal::degrees p_pitch_min,
               hal::degrees p_pitch_max)
  : m_yaw_servo(p_yaw_servo)
  , m_pitch_servo(p_pitch_servo)
  , m_yaw_min(p_yaw_min)
  , m_yaw_max(p_yaw_max)
  , m_pitch_min(p_pitch_min)
  , m_pitch_max(p_pitch_max)
{
  m_curr_yaw_servo_angle = (m_yaw_min + m_yaw_max) / 2.0f;
  m_curr_pitch_servo_angle = (m_pitch_min + m_pitch_max) / 2.0f;

  m_yaw_servo->position(m_curr_yaw_servo_angle);
  m_pitch_servo->position(m_curr_pitch_servo_angle);
}

void gimbal::set_target(hal::degrees p_x_deg, hal::degrees p_y_deg)
{
  set_yaw_target(p_x_deg);
  set_pitch_target(p_y_deg);
}

void gimbal::set_yaw_target(hal::degrees p_yaw_deg)
{
  m_curr_yaw_servo_angle = std::clamp(p_yaw_deg, m_yaw_min, m_yaw_max);
  m_yaw_servo->position(m_curr_yaw_servo_angle);
}

void gimbal::set_pitch_target(hal::degrees p_pitch_deg)
{
  m_target_pitch = std::clamp(p_pitch_deg, m_pitch_min, m_pitch_max);
}

hal::degrees gimbal::get_yaw_angle() const
{
  return m_curr_yaw_servo_angle;
}

hal::degrees gimbal::get_pitch_angle() const
{
  return m_filtered_sensor_pitch;
}

void gimbal::update_pitch_servo(float p_delta_time,
                                hal::accelerometer::read_t const& p_accel,
                                hal::gyroscope::read_t const& p_gyro)
{
  float alpha = m_settings.tau / (m_settings.tau + p_delta_time);

  // Estimate pitch from accelerometer
  float angle_pitch_deg =
    atan2f(p_accel.x, sqrtf(p_accel.y * p_accel.y + p_accel.z * p_accel.z)) *
    180.0f / std::numbers::pi;

  // Complementary filter: fuse gyro integration with accel estimate
  m_filtered_sensor_pitch =
    alpha * (m_filtered_sensor_pitch + p_gyro.y * p_delta_time) +
    (1.0f - alpha) * angle_pitch_deg;

  // Direct compensation: subtract tilt from center to keep camera level
  float new_angle_pos = m_target_pitch - m_filtered_sensor_pitch;

  new_angle_pos = std::clamp(new_angle_pos, m_pitch_min, m_pitch_max);
  m_pitch_servo->position(new_angle_pos);
  m_curr_pitch_servo_angle = new_angle_pos;
}
}  // namespace sjsu::hub
