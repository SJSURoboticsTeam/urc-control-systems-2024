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
  float tau = 0.75f;  // For the complementary filter

  float kP_pitch = 1.2f;
  float kI_pitch = 0.6f;
  float kD_pitch = 0.0f;

  float kP_yaw = 0.0f;
  float kI_yaw = 0.0f;
  float kD_yaw = 0.0f;

  hal::degrees max_servo_step =
    5;  // Maximum step the servo can take during PID
};

struct pid_error
{
  float eP;
  float eI;
  float eD;
};

class gimbal
{
public:
  gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo16> p_x_servo,
         hal::v5::strong_ptr<hal::actuator::rc_servo16> p_y_servo,
         hal::degrees p_min_angle,
         hal::degrees p_max_angle);

  void set_target(hal::degrees p_x_deg, hal::degrees p_y_deg);
  void set_yaw_target(hal::degrees p_yaw_deg);
  void set_pitch_target(hal::degrees p_pitch_deg);
  void update_y_servo(float p_delta_time,
                      sensor_axis const& p_accel,
                      sensor_axis const& p_gyro);

  uint8_t get_x_angle() const;
  uint8_t get_y_angle() const;

private:
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_x_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo16> m_y_servo;

  bool m_first_update = true;

  hal::degrees m_min_angle, m_max_angle;

  hal::degrees m_curr_y_servo_angle, m_curr_x_servo_angle;

  float m_tar_pitch, m_filtered_pitch;

  struct pid_error m_error;
  gimbal_control_settings m_settings;
};
}  // namespace sjsu::hub