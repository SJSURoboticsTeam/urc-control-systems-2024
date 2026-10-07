#pragma once
#include <bldc_servo.hpp>

using sec = float;

namespace sjsu::perseus {

// track
constexpr bldc_perseus::PID_settings track_pid_settings = {
  .kp = 0.04,
  .ki = 0.00,
  .kd = 0.00,
};
constexpr bldc_perseus::physical_servo_values track_physical_servo_values = {
  .gear_ratio = 16915.5,  // 751.8 * 1 / 2 * 360 / 8 (for mm)
  .angle_offset = 0,
  .fight_gravity = 0,
  .high_clamped_value = 0.3,
  .low_clamped_value = -0.3,
  .clockwise_positive = false
};

// shoulder
constexpr bldc_perseus::PID_settings shoulder_pid_settings = {
  .kp = 0.5,
  .ki = 0.00,
  .kd = 0.007,
};
constexpr bldc_perseus::physical_servo_values shoulder_physical_servo_values = {
  .gear_ratio = 73935.4,  // 5281.1 * 28 / 2
  .angle_offset = 0,
  .fight_gravity = 0,
  .high_clamped_value = 0.3,
  .low_clamped_value = -0.3,
  .clockwise_positive = false
};

// elbow
constexpr bldc_perseus::PID_settings elbow_pid_settings = {
  .kp = 0.01,
  .ki = 0.00,
  .kd = 0.005,
};
constexpr bldc_perseus::physical_servo_values elbow_physical_servo_values = {
  .gear_ratio = 5281.1,  // 5281.1 * 2 / 2
  .angle_offset = 0,
  .fight_gravity = 0.15,
  .high_clamped_value = 0.1,
  .low_clamped_value = -0.3,
  .clockwise_positive = false
};

// wrist_left
constexpr bldc_perseus::PID_settings wrist_left_pid_settings = {
  .kp = 0.005,
  .ki = 0.00,
  .kd = 0.00,
};
constexpr bldc_perseus::physical_servo_values wrist_left_physical_servo_values = {
  .gear_ratio = 2640.55,  // 5281.1 * 1 / 2
  .angle_offset = 0,
  .fight_gravity = 0.2,
  .high_clamped_value = 0.3,
  .low_clamped_value = -0.3,
  .clockwise_positive = false
};

// wrist_right
constexpr bldc_perseus::PID_settings wrist_right_pid_settings = {
  .kp = 0.005,
  .ki = 0.00,
  .kd = 0.00,
};
constexpr bldc_perseus::physical_servo_values wrist_right_physical_servo_values = {
  .gear_ratio = 2640.55,  // 5281.1 * 1 / 2
  .angle_offset = 0,
  .fight_gravity = 0.2,
  .high_clamped_value = 0.3,
  .low_clamped_value = -0.3,
  .clockwise_positive = true
};

// wrist_right
constexpr bldc_perseus::PID_settings end_effector_pid_settings = {
  .kp = 0.05,
  .ki = 0.00,
  .kd = 0.05,
};
constexpr bldc_perseus::physical_servo_values end_effector_physical_servo_values = {
  .gear_ratio = 73935.4,  // 5281.1 * 28 / 2
  .angle_offset = 0,
  .fight_gravity = 0,
  .high_clamped_value = 0.45,
  .low_clamped_value = -0.45,
  .clockwise_positive = false
};

}  // namespace sjsu::perseus
