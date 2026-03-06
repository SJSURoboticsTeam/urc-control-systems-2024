#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
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
  m_active_power = 0; 
  m_target = { 
    .position = 0, 
    .power = 0.0f , 
    .velocity = 0
  };
  m_prev_encoder_value = bldc_perseus::read_angle();
  m_PID_prev_velocity_values = {
    .integral = 0, 
    .last_error = 0, 
    .prev_timestamp = 0.0 
  };
  m_PID_prev_position_values = { 
    .integral = 0,              
    .last_error = 0,
    .prev_timestamp = 0.0 
  };
  // default 
  m_servo_values = {
    .gear_ratio = 0.01, 
    .angle_offset = 0.01, 
    .fight_gravity = 0.01, 
    .high_clamped_value = 0.01, 
    .low_clamped_value = -0.01 
  }; 
// CHANGE SERVO
  m_actual_position = m_servo_values.angle_offset;  
  m_prev_joint_position = 0; 
  m_active_action = 0x000; 
}

void bldc_perseus::set_target_position(float target_position)
{
  m_target.position = target_position;
}

float bldc_perseus::get_target_position()
{
  return m_target.position;
}

// float bldc_perseus::get_reading_position()
// {
//   m_reading.position = bldc_perseus::read_angle();
//   return m_reading.position;
// }

// void bldc_perseus::set_reading_position(float position)
// {
//   m_reading.position = position;
//   auto console = resources::console(); 
//   hal::print(*console, "\nHH\n"); 
// }

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

  const hal::u64 now_time = m_clock->uptime();
  const hal::u64 dt_time = now_time - m_last_clock_check;

  const float  dt_sec = static_cast<float>(dt_time) / static_cast<float>(m_clock->frequency());

  if (dt_sec <= 0.0f){
    return m_reading.velocity;
  }
  
  const hal::degrees current_position = bldc_perseus::read_angle();
  const float d_theta = (current_position - m_prev_encoder_value);
  
  m_reading.velocity = d_theta / dt_sec;

  m_prev_encoder_value = current_position;
  m_last_clock_check = now_time;
  
  return m_reading.velocity;

}

float bldc_perseus::get_power() {
  return m_active_power;
}

void bldc_perseus::set_power(float power) {
  m_active_power = power; 
  m_h_bridge->power(m_active_power);
}

void bldc_perseus::set_active_action(uint32_t action) {
  m_active_action = action; 
} 

uint32_t bldc_perseus::get_active_action() {
  return m_active_action; 
}

void bldc_perseus::stop()
{
  m_active_power = 0; 
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
}

hal::degrees bldc_perseus::read_angle() {
  return (m_encoder->read().angle / m_servo_values.gear_ratio); 
}

void bldc_perseus::update_velocity(bool from_scratch) 
{
  // TODO : implement velocity PID control
  if (from_scratch) m_PID_prev_velocity_values.integral = 0; 
}

void bldc_perseus::reset_time()
{
  m_PID_prev_velocity_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_timestamp = 0.0 };
  m_PID_prev_position_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_timestamp = 0.0 };
  m_last_clock_check = m_clock->uptime();
}

void bldc_perseus::set_pos_clamped_power(float power)
{
  m_servo_values.high_clamped_value = power; 
}
float bldc_perseus::get_pos_clamped_power() {
  return m_servo_values.high_clamped_value; 
}
void bldc_perseus::set_neg_clamped_power(float power)
{
  m_servo_values.low_clamped_value = power; 
}
float bldc_perseus::get_neg_clamped_power() {
  return m_servo_values.low_clamped_value; 
}

hal::time_duration bldc_perseus::get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}
// position 
void bldc_perseus::update_position(bool from_scratch) 
{
  auto console = resources::console(); 
  // pid portion
  set_actual_position(); 
  float error = m_target.position - m_actual_position;
  sec curr_time = hal_time_duration_to_sec(get_clock_time(*m_clock));
  sec dt = curr_time - m_PID_prev_position_values.prev_timestamp;
  if (from_scratch) { 
    m_PID_prev_position_values.integral = 0.0f; 
  }
  m_PID_prev_position_values.integral += error * dt; 
  float derivative = (error - m_PID_prev_position_values.last_error) / dt; 
  float pTerm = m_reading_position_settings.kp * error; 
  float iTerm  = m_reading_position_settings.ki * m_PID_prev_position_values.integral; 
  float dTerm = m_reading_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_timestamp = curr_time;
  float pid_sum = pTerm + iTerm + dTerm;
  // feed forward 
  float feedforward = bldc_perseus::position_feedforward(); 
  // apply 
  float projected_power = pid_sum + feedforward; 
  // CHANGE SERVO
  // // use actual position here once can be communicated/calculated via can 
  // if (m_actual_position < 0) 
  // { 
  //   projected_power = std::clamp(projected_power, -1 * m_clamped_power, m_clamped_power); 
  // }
  // else { 
  //   projected_power = std::clamp(projected_power, -1 * m_clamped_power, m_clamped_power);
  // }
  projected_power = std::clamp(projected_power, m_servo_values.low_clamped_value, m_servo_values.high_clamped_value);
  // if (m_actual_position > 0) {
  //   float t = m_servo_values.low_clamped_value; 
  //   m_servo_values.low_clamped_value = m_servo_values.high_clamped_value * -1; 
  //   m_servo_values.high_clamped_value = t * -1; 
  // }
  hal::print<128>(*console, "Target: %f, Position: %f, Error: %f, pid: %f, projected: %f\n", m_target.position, m_actual_position, error, pid_sum, projected_power); 
  m_active_power = projected_power; 
  m_h_bridge->power(m_active_power);
}

// use actual position here once can be communicated/calculated via can 
float bldc_perseus::position_feedforward() 
{
  return std::sin(std::numbers::pi/180 * m_actual_position) 
    * m_servo_values.fight_gravity; 
}

void bldc_perseus::set_prev_joint_position(float prev_joint_pos) {
  m_prev_joint_position = prev_joint_pos; 
}

float bldc_perseus::get_prev_joint_position() {
  return m_prev_joint_position; 
}

void bldc_perseus::set_angle_offset(float angle_offset) {
  m_servo_values.angle_offset = angle_offset; 
}

float bldc_perseus::get_angle_offset() {
  return m_servo_values.angle_offset; 
}

void bldc_perseus::set_actual_position() {
  m_actual_position = read_angle() + m_servo_values.angle_offset + m_prev_joint_position; 
}

float bldc_perseus::get_actual_position() {
  return m_actual_position; 
}

void bldc_perseus::set_servo_values(servo_values p_servo_values) {
  m_servo_values = p_servo_values; 
}

void bldc_perseus::periodic_action(bool new_action) {
  auto console = resources::console(); 
  switch (static_cast<can_perseus::action>(m_active_action)) {
    case can_perseus::action::homing: {
      home_encoder(); 
      break; 
    }
    case can_perseus::action::set_position_target: {
      update_position(new_action); 
      hal::print<128>(*console, "Target pos: %f", get_target_position()); 
      hal::delay(*m_clock, 1000ms);
      break;
    }
    default:
      break; 
  }
}


}// namespace sjsu::perseus
