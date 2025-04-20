#pragma once
#include "drive_configuration.hpp"
#include "steering_module.hpp"

#include <array>
#include <cstddef>

using namespace std::chrono_literals;
using namespace hal::literals;

namespace sjsu::drive {

// Multiply this with the desired angle to get the correct angle to steer the
// motors.
/**
 * @brief The steering motors are scaled, meaning commanding them to rotate to
 * 60 degrees will make them rotate to around 44 degrees. Multiply this number
 * to rotate the steering motors to the correct angles.
 *
 * I have no idea why. This value was calculated by trial and error.
 */
constexpr float angle_correction_factor = 1.3625;

/**
 * @brief Maximum speed sendable from mission control. The maximum is `100`
 * since this is a percentage.
 *
 * @note THIS IS NOT THE MAX SPEED IN RPMS.  If we want to change this to rpms,
 * we should move away from using `hal::motor.`
 * @note To change the actual maximum speed of the motors, you have to change it
 * in the `platforms/lpc4078.cpp`
 */
constexpr float k_max_speed = 100;

/**
 * @brief Defines how "fast" the drive configuration manager will try to achieve
 * the target configuration.
 *
 * @note The units are in 1/seconds (hertz), but this is not that useful.
 * These are the kPs in PID control.
 */
constexpr drive_configuration_sensitivity config_sensitivity = {
  .steering_angle = 1,
  .wheel_heading = 10,
  .wheel_speed = 20,
};

/**
 * @brief This is how large of a change the drive configuration manager is
 * allowed to make. If the requested change for either `steering_angle`,
 * `wheel_heading`, or `wheel_speed` is larger than theses max deltas, then the
 * change will be clamped.
 *
 * @note Units are in degrees/second, in order to keep unit consistency.
 */
constexpr drive_configuration_rate config_max_delta = {
  .steering_angle_rate = 40,  // degrees/sec
  .wheel_heading_rate = 10,   // degrees/sec
  .wheel_speed_rate = 50,     // units/sec
};

constexpr start_wheel_setting front_left_wheel_setting = {
  .steer_id = 0x14c,
  .prop_id = 0x150,
  .geer_ratio = 36.0f,
  .reversed = false,
  .homing_offset = 90,
  .homing_angle = 0,
  .max_speed = 5,
};

constexpr start_wheel_setting front_right_wheel_setting = {
  .steer_id = 0x143,
  .prop_id = 0x151,
  .geer_ratio = 36.0f,
  .reversed = false,
  .homing_offset = 90,
  .homing_angle = 0,
  .max_speed = 5,
};

constexpr start_wheel_setting back_left_wheel_setting = {
  .steer_id = 0x148,
  .prop_id = 0x152,
  .geer_ratio = 36.0f,
  .reversed = true,
  .homing_offset = 90,
  .homing_angle = 0,
  .max_speed = 5,
};

constexpr start_wheel_setting back_right_wheel_setting = {
  .steer_id = 0x14f,
  .prop_id = 0x153,
  .geer_ratio = 36.0f,
  .reversed = true,
  .homing_offset = 90,
  .homing_angle = 0,
  .max_speed = 5,
};

};  // namespace sjsu::drive