#include "../include/bldc_servo.hpp"
#include "h_bridge.hpp"
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<drivers::h_bridge> p_hbridge,
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

void bldc_perseus::set_target_velocity(hal::u16 target_velocity)
{
  // we want to try to arrive at velocity
  m_target.velocity = target_velocity;
  float diff = m_target.velocity - m_current.velocity;
  // lerp to target velocity (idk if this is called lerping but whatev
  m_current.velocity += std::clamp(diff, -1 * m_clamped_accel, m_clamped_accel);
  h_bridge->power(m_current.velocity);
}

hal::u16 bldc_perseus::get_current_velocity()
{
  return m_current.velocity;
}

hal::u16 bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}

void bldc_perseus::reset_encoder()
{
  // make the encoder eading value 0
  // we don't have ability to change the value insdie the driver encoder
  
  m_current.position = 0;
  
}
}  // namespace sjsu::perseus
