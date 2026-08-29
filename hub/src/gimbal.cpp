#include <gimbal.hpp>
#include <numbers>
namespace sjsu::hub {

gimbal::gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_x_servo,
               hal::v5::strong_ptr<hal::actuator::rc_servo16> p_y_servo,
               hal::degrees p_min_angle,
               hal::degrees p_max_angle,
               hal::degrees p_pitch_min,
               hal::degrees p_pitch_max)
  : m_x_servo(p_x_servo)
  , m_y_servo(p_y_servo)
  , m_min_angle(p_min_angle)
  , m_max_angle(p_max_angle)
  , m_pitch_min(p_pitch_min)
  , m_pitch_max(p_pitch_max)
{
  m_curr_x_servo_angle = (m_min_angle + m_max_angle) / 2.0f;
  m_curr_y_servo_angle = (m_pitch_min + m_pitch_max) / 2.0f;

  m_x_servo->position(m_curr_x_servo_angle);
  m_y_servo->position(m_curr_y_servo_angle);

  m_tar_pitch = 0.0f;
  m_filtered_pitch = 0.0f;
}

void gimbal::set_target(hal::degrees p_x_deg, hal::degrees p_y_deg)
{
  set_yaw_target(p_x_deg);
  set_pitch_target(p_y_deg);
}

void gimbal::set_yaw_target(hal::degrees p_yaw_deg)
{
  m_curr_x_servo_angle = std::clamp(p_yaw_deg, m_min_angle, m_max_angle);
  m_x_servo->position(m_curr_x_servo_angle);
}

void gimbal::set_pitch_target(hal::degrees p_pitch_deg)
{
  hal::degrees clamped_pitch =
    std::clamp(p_pitch_deg, m_pitch_min, m_pitch_max);

  // Convert external angle to offset from level
  // pitch_center -> 0 offset
  m_tar_pitch = clamped_pitch - ((m_pitch_min + m_pitch_max) / 2.0f);
}

uint8_t gimbal::get_x_angle() const
{
  return static_cast<uint8_t>(m_curr_x_servo_angle);
}

uint8_t gimbal::get_y_angle() const
{
  return static_cast<uint8_t>(m_curr_y_servo_angle);
}

void gimbal::update_y_servo(float p_delta_time,
                            sensor_axis const& p_accel,
                            sensor_axis const& p_gyro)
{
  float alpha = m_settings.tau / (m_settings.tau + p_delta_time);

  // Estimate pitch from accelerometer
  float angle_pitch_deg =
    atan2f(p_accel.x,
           sqrtf(p_accel.y * p_accel.y + p_accel.z * p_accel.z)) *
    180.0f / std::numbers::pi;

  // Complementary filter: fuse gyro integration with accel estimate
  m_filtered_pitch = alpha * (m_filtered_pitch + p_gyro.y * p_delta_time) +
                     (1.0f - alpha) * angle_pitch_deg;

  // Direct compensation: subtract tilt from center to keep camera level
  // m_tar_pitch allows MC to offset the camera from level
  float pitch_center = (m_pitch_min + m_pitch_max) / 2.0f;
  float new_angle_pos = pitch_center - m_filtered_pitch + m_tar_pitch;

  new_angle_pos = std::clamp(new_angle_pos, m_pitch_min, m_pitch_max);
  m_y_servo->position(new_angle_pos);
  m_curr_y_servo_angle = new_angle_pos;
}
}  // namespace sjsu::hub