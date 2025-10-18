#pragma once
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
// #include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

namespace sjsu::drivers {
class gimbal
{
public:
  struct rotation_axis
  {
    float pitch, roll, yaw;
  };

  gimbal(hal::v5::strong_ptr<hal::i2c> p_i2c,
         hal::v5::strong_ptr<hal::steady_clock> p_clock,
         hal::v5::strong_ptr<hal::serial> p_terminal);

  void set_filter_tau(float initTau);
  void update_gimbal();  // Call this function to update the servo

  void set_target_axis(float initTargetPitch,
                       float initTargetRoll,
                       float initTargetYaw);

  void set_pid_pitch(float kp, float ki, float kd);

private:
  void calculate_rotation_axis(float initDt);
  void calculate_control_var(float initDt);

  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::serial> m_terminal;

  // Rotation Axis Target States
  struct rotation_axis target_axis = {0, 0, 0};

  // Rotatation Axis Current States
  struct rotation_axis current_axis = {0, 0, 0}; 

  // Tunable variables for PID
  float Kp_pitch = 1.2f;  // Proportional Gain
  float Ki_pitch = 0.6f;  // Integral Gain
  float Kd_pitch = 0.0f;  // Derivative Gain
  
  float max_step_deg = 30.0f;  // Cap on how much the servo can move every iteration
  float authority_clamp_deg = 85.0f; //Cap on how much the servo cane move from the center of 90 degrees
  float dt = 0.0f;

  // Error values for PID
  float e_integral_pitch = 0.0f;
  float previous_error_pitch = 0.0f;

  // Tuning variable for Complementary Filter
  float tau = 0.75f;
  float alpha = 0.0f;
};
}  // namespace sjsu::drivers