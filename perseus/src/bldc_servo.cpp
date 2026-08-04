#include <libhal/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
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
    .low_clamped_value = -0.01, 
    .flipped_direction = false
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
  hal::u64 time_t0 = m_clock->uptime();
  hal::degrees pos_p0 = read_angle(); 
  hal::delay(*m_clock, 100ms); 
  hal::u64 time_t1 = m_clock->uptime(); 
  hal::degrees pos_p1 = read_angle(); 
  float dt = static_cast<float>(time_t1 - time_t0) / m_clock->frequency(); 
  float dp_over_dt = (pos_p1 - pos_p0) / dt; 
  return dp_over_dt; 
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
    sec_to_hal_time_duration(1.0f / p_clock.frequency());
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
  float p_term = m_reading_position_settings.kp * error; 
  float i_term  = m_reading_position_settings.ki * m_PID_prev_position_values.integral; 
  float d_term = m_reading_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_timestamp = curr_time;
  float pid_sum = p_term + i_term + d_term;
  // feed forward 
  float feedforward = bldc_perseus::position_feedforward(); 
  // apply 
  float projected_power = pid_sum + feedforward; 
  projected_power = std::clamp(projected_power, m_servo_values.low_clamped_value, m_servo_values.high_clamped_value);
  hal::print<128>(*console, "Target: %f, Position: %f, Error: %f, pid: %f, projected: %f\n", m_target.position, m_actual_position, error, pid_sum, projected_power); 
  m_active_power = projected_power; 
  m_h_bridge->power(m_active_power);
}

// use actual position here once can be communicated/calculated via can 
float bldc_perseus::position_feedforward() 
{
  return std::sin(static_cast<float>(std::numbers::pi/180) * m_actual_position) 
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
  m_actual_position = read_angle() + m_servo_values.angle_offset; 
  if (m_servo_values.flipped_direction) {
    m_actual_position = m_actual_position - m_prev_joint_position; 
  }
  else {
    m_actual_position = m_actual_position + m_prev_joint_position; 
  }
}

float bldc_perseus::get_actual_position() {
  return m_actual_position; 
}

void bldc_perseus::set_servo_values(servo_values p_servo_values) {
  m_servo_values = p_servo_values; 
}

void bldc_perseus::periodic_action(bool new_action) {
  switch (static_cast<can_perseus::action>(m_active_action)) {
    case can_perseus::action::homing: {
      home_encoder(); 
      break; 
    }
    case can_perseus::action::set_position_target: {
      update_position(new_action); 
      break;
    }
    default:
      break; 
  }
}


}// namespace sjsu::perseus
