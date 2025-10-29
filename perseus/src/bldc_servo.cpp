#include "../include/bldc_servo.hpp"
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>

#include <sys/types.h>
namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : h_bridge(p_hbridge)
  , m_encoder(p_encoder)
{
  m_current = {
    .position = 0,
    .velocity = 0,  // 0-100
  };
  m_target = { .position = 0, .velocity = 0 };
  m_clamped_speed =
    0.3;  // 40*0.3 = 12V which is the maximum this servo can be drivern
  m_clamped_accel = 0.1;  // if we are currently at a velocity of +0.2, we must
                          // not immediately change our current velocity to
                          // -0.2, even if our target velocity changes to -0.2
  current_encoder_value = m_encoder->read().angle;
}

void bldc_perseus::set_target_position(hal::u16 target_position)
{
  m_target.position = target_position;
}

hal::u16 bldc_perseus::get_target_position()
{
  return m_target.position;
}

hal::u16 bldc_perseus::get_current_position()
{

  m_current.position = static_cast<hal::u16>(m_encoder->read().angle);
  return m_current.position;
}

void bldc_perseus::set_target_velocity(hal::i16 target_velocity)
{
  m_target.velocity = target_velocity;
}

hal::u16 bldc_perseus::get_current_velocity_in_tps()
{
  return m_current.velocity;
}

hal::u16 bldc_perseus::get_current_velocity_percentage()
{
  return m_current.velocity;
}
void bldc_perseus::set_current_velocity(hal::i16 current_velocity)
{
  m_current.velocity = current_velocity;
}
hal::u16 bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}
void bldc_perseus::set_clamped_speed(hal::u16 clamped_speed)
{
  m_clamped_speed = clamped_speed;
}
void bldc_perseus::update_pid_position(PID_settings settings)
{
  m_current_position_settings = settings;
}
void bldc_perseus::update_pid_velocity(PID_settings settings)
{
  m_current_velocity_settings = settings;
}
void bldc_perseus::home_encoder()
{
  current_encoder_value = m_encoder->read().angle;
  m_current.position = 0;
}

void bldc_perseus::update()
{
  auto position_error = m_target.position - m_current.position;
  auto velocity_error = m_target.velocity - m_current.velocity;
  total_position_error += position_error;
  total_velocity_error += velocity_error;
  // PID calculations
  auto p_term_position = m_current_position_settings.kp * position_error;
  auto i_term_position = m_current_position_settings.ki * position_error;
  auto d_term_position = m_current_position_settings.kd * (position_error - last_position_error);

  auto p_term_velocity = m_current_velocity_settings.kp * velocity_error;
  auto i_term_velocity = m_current_velocity_settings.ki * velocity_error;
  auto d_term_velocity =
    m_current_velocity_settings.kd * (velocity_error - last_velocity_error);

  auto output_position = p_term_position + i_term_position + d_term_position;
  auto output_velocity = p_term_velocity + i_term_velocity + d_term_velocity;

  // servo->h_bridge->power()
}
}  // namespace sjsu::perseus
