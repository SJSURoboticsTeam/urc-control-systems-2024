//template from drivers/include/h_bridge.hpp

#pragma once
#include <array>
#include <libhal/pointers.hpp>
#include <libhal/motor.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>


namespace sjsu::perseus {

struct PidSettings // A structure to hold PID (Proportional, Integral, Derivative)
{
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

class perseus_bldc
{
public:
  perseus_bldc() = default;  
  
  void set_encoder_position(hal::degrees position);
  void set_percent_voltage(float percent);
  void set_pid(const PidSettings& p_settings);
  hal::degrees get_position();
  hal::degrees get_velocity();
  void set_position(hal::degrees degrees);
  void set_velocity(float rpm);
  // void set_feedforward(float );
  void reset_factor();
  void set_gear_ratio(float p_ratio);

};
}  // namespace sjsu::drivers
