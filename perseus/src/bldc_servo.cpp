#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

#include <bldc_servo.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;

namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : m_h_bridge(p_hbridge)
  , m_encoder(p_encoder)
  , m_clock(resources::clock())
{
  m_last_clock_check = m_clock->uptime(); 

  m_reading = {
    .position = 0,
    .power = 0, 
    .velocity = 0,  
  };
  m_target = { 
    .position = 0, 
    .power = 0.0f , 
    .velocity = 0
  };
  m_clamped_power = 0.3;
  m_prev_encoder_value = bldc_perseus::read_angle();
  m_PID_prev_velocity_values = {
    .integral = 0, 
    .last_error = 0, 
    .prev_dt_time = 0.0 
  };
  m_PID_prev_position_values = { 
    .integral = 0,              
    .last_error = 0,
    .prev_dt_time = 0.0 
  };
  // elbow 
  m_servo_values = {
    .gear_ratio = 10562.2, // 5281.1 * 2
    .feedforward_clamp = 0.2, 
    .length = 0.4826, 
    .angle_offset = -20, 
    .weight_beam = 1000, 
    .weight_end = 600 
  }; 
  // // shoulder 
  // m_servo_values = {
  //   .gear_ratio = 147870.8, // 5281.1 * 28
  //   .feedforward_clamp = 0, 
  //   .length = 0.5715, 
  //   .angle_offset = -20, 
  //   .weight_beam = 1600, 
  //   .weight_end = 1600 
  // }; 
  // // wrist 
  // m_servo_values = {
  //   .gear_ratio = 5281.1, // 5281.1 * 1
  //   .feedforward_clamp = 0.2,
  //   .length = 0.762, 
  //   .angle_offset = 0, 
  //   .weight_beam = 500, 
  //   .weight_end = 100 
  // }; 
// // track 
  // m_servo_values = {
  //   .gear_ratio = 751.8, // 751.8 * 1
  //   .feedforward_clamp = 0,
  //   .length = 0, 
  //   .angle_offset = 0, 
  //   .weight_beam = 0, 
  //   .weight_end = 0 
  // }; 
}

void bldc_perseus::set_target_position(float target_position)
{
  m_target.position = target_position;
}

float bldc_perseus::get_target_position()
{
  return m_target.position;
}

float bldc_perseus::get_reading_position()
{
  m_reading.position = bldc_perseus::read_angle();
  return m_reading.position;
}

void bldc_perseus::set_target_velocity(float target_velocity)
{
  m_target.velocity = target_velocity;
}

float bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}
float bldc_perseus::get_reading_velocity()
{
  // TODO! 

  const size_t now_time = m_clock->uptime();
  const size_t dt_time = now_time - m_last_clock_check;

  const float  dt_sec = static_cast<float>(dt_time) / static_cast<float>(m_clock->frequency());

  if (dt_sec <= 0.0f){
    return m_reading.velocity;
  }
  
  const hal::degrees current_position = bldc_perseus::read_angle();
  const float d_theta = (current_position - m_prev_encoder_value).to<float>();
  
  m_reading.velocity = d_theta / dt_sec;

  m_prev_encoder_value = current_position;
  m_last_clock_check = now_time;
  
  return m_reading.velocity;

}

void bldc_perseus::set_power(float power) {
  m_h_bridge->power(power);
}


void bldc_perseus::stop()
{
  m_reading.power = 0.0f;
  m_h_bridge->power(0.0f);
}


bldc_perseus::PID_settings bldc_perseus::get_pid_settings()
{
  return m_reading_position_settings;
}
void bldc_perseus::update_pid_position(PID_settings settings)
{
  m_reading_position_settings = settings;
}
void bldc_perseus::update_pid_velocity(PID_settings settings)
{
  m_reading_velocity_settings = settings;
}
void bldc_perseus::home_encoder()
{
  // TODO!
  home_encoder_value = read_angle();
  m_reading.position = 0;
}

hal::degrees bldc_perseus::read_angle() {
  return m_encoder->read().angle * m_servo_values.gear_ratio; 
}

void bldc_perseus::update_velocity() 
{
  // TODO : implement velocity PID control
}

void bldc_perseus::reset_time()
{
  m_PID_prev_velocity_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
  m_PID_prev_position_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
  m_last_clock_check = m_clock->uptime();
}

void bldc_perseus::set_pid_clamped_power(float power)
{
  m_clamped_power = power; 
}
float bldc_perseus::get_pid_clamped_power() {
  return m_clamped_power; 
}

hal::time_duration bldc_perseus::get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}
// position 
void bldc_perseus::update_position() 
{
  // pid portion
  m_reading.position = bldc_perseus::read_angle();
  float error = m_target.position - m_reading.position;
  sec curr_time = hal_time_duration_to_sec(get_clock_time(*m_clock));
  sec dt = curr_time - m_PID_prev_position_values.prev_dt_time;
  m_PID_prev_position_values.integral += error * dt; 
  float derivative = (error - m_PID_prev_position_values.last_error) / dt; 
  float pTerm = m_reading_position_settings.kp * error; 
  float iTerm  = m_reading_position_settings.ki * m_PID_prev_position_values.integral; 
  float dTerm = m_reading_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_dt_time = curr_time;
  float pid_sum = pTerm + iTerm + dTerm;
  // feed forward 
  float feedforward = bldc_perseus::position_feedforward(); 
  // apply 
  float projected_power = pid_sum + feedforward; 
  // use actual position here once can be communicated/calculated via can 
  if (m_reading.position < 0) 
  { 
    projected_power = std::clamp(projected_power, -1 * m_clamped_power, -0.1f * m_clamped_power); 
  }
  else { 
    projected_power = std::clamp(projected_power, -1 * m_clamped_power, m_clamped_power);
  }
  m_reading.power = projected_power; 
  m_h_bridge->power(m_reading.power);
}

// use actual position here once can be communicated/calculated via can 
float bldc_perseus::position_feedforward() 
{
  return std::sin(std::numbers::pi/180 * (m_reading.position + m_servo_values.angle_offset)) 
    * m_servo_values.feedforward_clamp; 
}

}// namespace sjsu::perseus
