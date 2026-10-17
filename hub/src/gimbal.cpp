#include <algorithm>
#include <gimbal.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/units.hpp>
#include <math_helpers.hpp>
#include <numbers>

namespace sjsu::hub {
clamped_servo::clamped_servo(
  hal::strong_ptr<hal::servo> p_servo,
  clamped_servo::settings p_settings)
  : m_servo{ p_servo }
  , m_settings{ p_settings }
{
}

void clamped_servo::driver_position(hal::degrees p_position)
{
  auto clamped_pos =
    std::clamp(p_position, m_settings.min_angle, m_settings.max_angle);
  m_servo->position(clamped_pos);
}

imu_angle_estimator::imu_angle_estimator(
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::v5::strong_ptr<hal::accelerometer> p_accel,
  hal::v5::strong_ptr<hal::gyroscope> p_gyro,
  imu_angle_estimator::settings p_settings)
  : m_clock{ p_clock }
  , m_accel{ p_accel }
  , m_gyro{ p_gyro }
  , m_settings{ p_settings }
  , m_estimated_aboutx{}
  , m_estimated_abouty{}
  , m_last_uptime{}
{
}

hal::degrees imu_angle_estimator::get_angle_aboutx()
{
  return m_estimated_aboutx;
}

hal::degrees imu_angle_estimator::get_angle_abouty()
{
  return m_estimated_abouty;
}

float imu_angle_estimator::delta_time()
{
  if (m_last_uptime == 0) {
    // 0 dt causes complementary filter to ignore gyro (which makes sense, as we
    // have no prior measurement for gyro)
    return 0;
  }

  hal::u64 now = m_clock->uptime();
  hal::u64 counts = now - m_last_uptime;
  m_last_uptime = now;

  hal::hertz counts_per_second = m_clock->frequency();
  float dt_in_sec = static_cast<float>(counts) * counts_per_second;

  return dt_in_sec;
}

void imu_angle_estimator::update()
{
  float dt = delta_time();
  float alpha = m_settings.tau / (m_settings.tau + dt);

  auto res_accel = m_accel->read();
  auto res_gyro = m_gyro->read();

  // estimate from accel
  float aboutx_deg = rad_to_deg(
    atan2f(res_accel.x,
           std::sqrt(res_accel.y * res_accel.y + res_accel.z * res_accel.z)));

  // complementary filter
  m_estimated_aboutx = alpha * (m_estimated_aboutx + res_gyro.x * dt) +
                       (1.0f - alpha) * aboutx_deg;

  float abouty_deg = rad_to_deg(atan2f(res_accel.y, res_accel.z));

  m_estimated_abouty = alpha * (m_estimated_abouty + res_gyro.y * dt) +
                       (1.0f - alpha) * abouty_deg;
}

gimbal::gimbal(hal::v5::strong_ptr<hal::servo> p_servo)
  : m_servo{ p_servo }
  , m_target_angle{}
{
}

void gimbal::set_target(hal::degrees p_angle)
{
  m_target_angle = p_angle;
}

void gimbal::update(hal::degrees p_angle_estimate)
{
  // Direct compensation: subtract tilt from center to level
  float new_angle_pos = m_target_angle - p_angle_estimate;
  m_servo->position(new_angle_pos);
}

mast_gimbal_controller::mast_gimbal_controller(components p_comp)
  : m_comp{ p_comp }
{
}

void mast_gimbal_controller::set_target(gimbal_target_request const& target)
{
  m_comp.mast_gimbal_abouty->set_target(target.mast_angle_y);
  m_comp.mast_servo_aboutx.position(target.mast_angle_x);
  m_comp.wheels_servo_aboutx.position(target.wheels_angle_x);
}

void mast_gimbal_controller::update()
{
  m_comp.angle_estimator->update();
  auto abouty_estimate = m_comp.angle_estimator->get_angle_abouty();
  m_comp.mast_gimbal_abouty->update(abouty_estimate);
}

}  // namespace sjsu::hub
