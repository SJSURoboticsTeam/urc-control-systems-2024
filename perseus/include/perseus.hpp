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
  explicit perseus_bldc(hal::v5::optional_ptr<hal::pwm16_channel> p_pin_a_low,
                    hal::v5::optional_ptr<hal::pwm16_channel> p_pin_b_low,
                    hal::v5::optional_ptr<hal::output_pin> p_pin_a_high,
                    hal::v5::optional_ptr<hal::output_pin> p_pin_b_high)
    : m_pin_a_low(p_pin_a_low)
    , m_pin_b_low(p_pin_b_low)
    , m_pin_a_high(p_pin_a_high)
    , m_pin_b_high(p_pin_b_high)

  GoBildaMotor(hal::pwm& p_pwm_a,
               hal::pwm& p_pwm_b,
               hal::interrupt_pin& p_encoder_a,
               hal::interrupt_pin& p_encoder_b,
               hal::steady_clock& p_clock);
  
  
  void set_encoder_position(hal::degrees position);
  void set_percent_voltage(hal::degrees percent);
  void set_pid_position(const PidSettings& p_settings);
  void set_pid_velocity(const PidSettings& p_settings);
  float get_position();
  float get_velocity();
  void set_position(hal::degrees degrees);
  void set_velocity(hal::degrees rpm);
  void set_feedforward();
  void reset_factor();
  void set_gear_ratio(hal::degrees p_ratio);

private:
  hal::v5::optional_ptr<hal::pwm16_channel> m_pin_a_low;
  hal::v5::optional_ptr<hal::pwm16_channel> m_pin_b_low;
  hal::v5::optional_ptr<hal::output_pin> m_pin_a_high;
  hal::v5::optional_ptr<hal::output_pin> m_pin_b_high;

  void driver_power(float p_power);
};
}  // namespace sjsu::drivers
