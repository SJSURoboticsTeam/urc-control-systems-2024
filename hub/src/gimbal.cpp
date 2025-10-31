#include "../include/gimbal.hpp"

namespace sjsu::hub {

gimbal::gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo> p_x_servo,
               hal::v5::strong_ptr<hal::actuator::rc_servo> p_y_servo,
               hal::v5::strong_ptr<hal::sensor::icm20948> p_icm,
               hal::degrees p_min_angle,
               hal::degrees p_max_angle)
  : m_x_servo(p_x_servo)
  , m_y_servo(p_y_servo)
  , m_icm(p_icm)
  , m_min_angle(p_min_angle)
  , m_max_angle(p_max_angle)
{
  float mid_angle = (m_min_angle + m_max_angle) / 2.0;

  // Setting starting positions for the servos
  curr_x_servo_angle = mid_angle;
  curr_y_servo_angle = mid_angle;

  // Setting the servos at some known location
  m_x_servo->position(curr_x_servo_angle);
  m_y_servo->position(curr_y_servo_angle);

  // Default
  tar_pitch = 0.0f;
  filtered_pitch = 0.0f;

  // Error
  error.Ep = 0.0f;
  error.Ei = 0.0f;
  error.Ed = 0.0f;
}

void gimbal::move_left(hal::degrees p_degree)
{
  // Based on how the servo motor is fixed, the left could be moving towards
  // m_min_angle or m_max_angle

  // Delete the one that it actually goes to

  // If left means going to m_min_angle
  if (curr_x_servo_angle - p_degree < m_min_angle)
    curr_x_servo_angle = m_min_angle;
  else
    curr_x_servo_angle -= p_degree;

  // If left means going to m_max_angle
  if (curr_x_servo_angle + p_degree > m_max_angle)
    curr_x_servo_angle = m_max_angle;
  else
    curr_x_servo_angle += p_degree;

  // Update the servo position
  m_x_servo->position(curr_x_servo_angle);
}

void gimbal::move_right(hal::degrees p_degree)
{
  // Based on how the servo motor is fixed, the right could be moving towards
  // m_min_angle or m_max_angle

  // Delete the one that it actually goes to

  // If left means going to m_min_angle
  if (curr_x_servo_angle - p_degree < m_min_angle)
    curr_x_servo_angle = m_min_angle;
  else
    curr_x_servo_angle -= p_degree;

  // If left means going to m_max_angle
  if (curr_x_servo_angle + p_degree > m_max_angle)
    curr_x_servo_angle = m_max_angle;
  else
    curr_x_servo_angle += p_degree;

  // Update the servo position
  m_x_servo->position(curr_x_servo_angle);
}

void gimbal::move_up(hal::degrees p_degree)
{
  // Based on how the servo motor is fixed, the up could be moving towards
  // m_min_angle or m_max_angle

  // Have to look into this cause I don't know the range of pitch given the ICM
  tar_pitch += p_degree;
}

void gimbal::move_down(hal::degrees p_degree)
{
  // Based on how the servo motor is fixed, the left could be moving towards
  // m_min_angle or m_max_angle

  // Have to look into this cause I don't know the range of pitch given the ICM
  tar_pitch -= p_degree;
}

void gimbal::update_y_servo(float p_delta_time)
{
  float alpha = gimbal_control_settings::tau /
                (gimbal_control_settings::tau +
                 p_delta_time);  // Used for the complementary filter

  // Reading the raw values from the sensor
  auto raw_accel = m_icm->read_acceleration();
  auto raw_gyro = m_icm->read_gyroscope();

  // Getting the pitch
  float angle_pitch_deg =
    atan2f(raw_accel.x,
           sqrtf(raw_accel.y * raw_accel.y + raw_accel.z * raw_accel.z)) *
    180.0f / gimbal_control_settings::pi;

  // Cleaning the pitch value from noise
  filtered_pitch = alpha * (filtered_pitch + raw_gyro.y * p_delta_time) +
                   (1.0f - alpha) * angle_pitch_deg;

  // Getting the error
  float curr_error = filtered_pitch - tar_pitch;

  // Checking if the current error will make the angle position set for the
  // servo out of bounds (the servo will fail)
  bool at_upper = (curr_y_servo_angle >= m_max_angle - 0.001f);
  bool at_lower = (curr_y_servo_angle >= m_min_angle + 0.001f);

  bool pushing_upper = at_upper && (curr_error > 0);
  bool pushing_lower = at_lower && (curr_error < 0);

  if (!pushing_upper && !pushing_lower)
    error.Ei += curr_error * p_delta_time;

  error.Ed = (curr_error - error.Ep) / p_delta_time;
  error.Ep = curr_error;

  // Calculating the offset to help the ICM sensor reach steady state
  // (tar_pitch)
  float control_var = (gimbal_control_settings::Kp_pitch * error.Ep) +
                      (gimbal_control_settings::Ki_pitch * error.Ei) +
                      (gimbal_control_settings::Kd_pitch * error.Ed);

  // Ensure that we aren't going above the servo's range
  control_var =
    std::clamp(control_var,
               m_min_angle + gimbal_control_settings::deg_tolerance,
               m_max_angle + gimbal_control_settings::deg_tolerance);

  float new_angle_pos = (m_min_angle + m_max_angle) / 2 + control_var;

  new_angle_pos = std::clamp(new_angle_pos, m_min_angle, m_max_angle);
  new_angle_pos =
    std::clamp(new_angle_pos,
               curr_y_servo_angle - gimbal_control_settings::max_servo_step,
               curr_y_servo_angle + gimbal_control_settings::max_servo_step);
  new_angle_pos = std::clamp(new_angle_pos, m_min_angle, m_max_angle);

  // Updating the servo position
  m_y_servo->position(new_angle_pos);
  curr_y_servo_angle = new_angle_pos;
}
}  // namespace sjsu::hub