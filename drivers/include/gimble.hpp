#pragma once
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
// #include <libhal/pwm.hpp>
#include <libhal-expander/pca9685.hpp>
#include <libhal-sensor/imu/mpu6050.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
class gimble
{
public:
  struct rotation_axis
  {
    float pitch, roll, yaw;
  };

  gimble(hal::v5::strong_ptr<hal::i2c> p_i2c,
         hal::v5::strong_ptr<hal::steady_clock> p_clock,
         hal::v5::strong_ptr<hal::serial> p_terminal);

  void set_alpha(float initTau, float initDt);
  void update_gimble();  // Call this function to update the servo

  void set_target_axis(float initTargetPitch,
                       float initTargetRoll,
                       float initTargeYaw);

private:
  void calculate_rotation_axis(float initDt);
  void calculate_control_var(float initDt);

  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::serial> m_terminal;

  // Rotation Axis Target States
  struct rotation_axis target_axis;

  // Rotatation Axis Current States
  struct rotation_axis current_axis;

  // Tunable variables for PID
  float const Kp_pitch;  // Proportional Gain
  float const Ki_pitch;  // Integral Gain
  float const Kd_pitch;  // Derivative Gain
  float const max_step;  // Cap on how much the servo can move every iteration
  float dt;

  // Error values for PID
  float e_integral_pitch;
  float previouis_error_pitch;

  // Tuning variable for Complementary Filter
  float const tau;
  float alpha;
};
}  // namespace sjsu::drivers