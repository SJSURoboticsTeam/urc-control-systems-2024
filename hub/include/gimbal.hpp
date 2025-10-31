#pragma once
#include <algorithm>
#include <cmath>

#include <libhal-actuator/rc_servo.hpp>    // For the two servos
#include <libhal-sensor/imu/icm20948.hpp>  // Need Gyro, Accel, and Compass
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

namespace sjsu::hub {

struct gimbal_control_settings
{
  static constexpr float tau = 0.75;  // For the complementary filter
  static constexpr float pi = 3.14159265f;

  // A PI Controller will be applied to the m_y_servo (up/down or pitch)
  static constexpr float Kp_pitch = 1.2f;
  static constexpr float Ki_pitch = 0.6f;
  static constexpr float Kd_pitch = 0.0f;

  // Some constants if Mission Controls ever wants to apply PID onto the
  // m_x_servo
  static constexpr float Kp_yaw = 0.0f;
  static constexpr float Ki_yaw = 0.0f;
  static constexpr float Kd_yaw = 0.0f;

  static constexpr hal::degrees max_servo_step =
    30.0;  // Maximum step the servo can take during PID
  static constexpr hal::degrees deg_tolerance = 5;
};

struct pid_error
{
  float Ep;  // Error on the proportional gain
  float Ei;  // Error on the integral gain
  float Ed;  // Error on the derivative gain
};

class gimbal
{
public:
  /**
   * @param p_x_servo the servo motor used to move the mast camera left and
   * right (no PID control on it)
   * @param p_y_servo the servo motor used to move the mast camera up and down
   * (yes PID control on it)
   * @param p_icm sensor needed to read the following: accelerometer, compass,
   * and gyroscope
   * @param p_min_angle minimum angle set inside the rc_servo_settings for both
   * the servo
   * @param p_max_angle maximum angle set inside the rc_servo_settings for both
   * the servo
   */
  gimbal(hal::v5::strong_ptr<hal::actuator::rc_servo> p_x_servo,
         hal::v5::strong_ptr<hal::actuator::rc_servo> p_y_servo,
         hal::v5::strong_ptr<hal::sensor::icm20948> p_icm,
         hal::degrees p_min_angle,
         hal::degrees p_max_angle);

  /**
   * @brief The function will move the gimbal towards the left by p_degrees (no
   * PID)
   *
   * @param p_degree the # of degrees to move the servo in the left direction
   */
  void move_left(hal::degrees p_degree);

  /**
   * @brief The function will move the gimbal towards the right by p_degrees (no
   * PID)
   *
   * @param p_degree the # of degrees to move the servo in the right direction
   */
  void move_right(hal::degrees p_degree);

  /**
   * @brief The function will update the target_pitch by p_degree and
   * update_y_servo will update the gimbal's oreitnation to match the
   *
   * @param p_degree the # of degrees to move the servo in the up direction
   */
  void move_up(hal::degrees p_degree);

  /**
   * @brief The function will update the target_pitch by p_degree and
   * update_y_servo will update the gimbal's oreitnation to match the
   *
   * @param p_degree the # of degrees to move the servo in the down direction
   */
  void move_down(hal::degrees p_degree);

  /**
   * @brief The function will update the m_y_servo where the error between it's
   * current pitch and target pitch is zero (PID)
   *
   * @param p_delta_time the time in seconds from the previous call of this
   * function and the current call of this function
   */
  void update_y_servo(float p_delta_time);

private:
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_x_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_y_servo;
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
  hal::degrees m_min_angle, m_max_angle;

  hal::degrees curr_y_servo_angle, curr_x_servo_angle;

  float tar_pitch, curr_pitch, filtered_pitch;  // For PID; in degrees
  // float tar_yaw, curr_pitch; // Incase Mission Control wants PID on the yaw
  // of the gimbal

  struct pid_error error;
};
}  // namespace sjsu::hub