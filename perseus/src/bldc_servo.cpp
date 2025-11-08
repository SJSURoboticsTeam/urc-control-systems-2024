#include "../include/bldc_servo.hpp"
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include "../hardware_map.hpp"
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <sys/types.h>
using namespace std::chrono_literals;
namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : m_h_bridge(p_hbridge)
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
  m_prev_encoder_value = m_encoder->read().angle;
  // inital/prev pid values 
  auto clock = resources::clock(); 
  m_last_clock_check = clock->uptime(); 
  m_PID_prev_velocity_values = {.integral = 0, .last_error = 0, .prev_dt_time = m_last_clock_check };
  m_PID_prev_position_values = { .integral = 0, .last_error = 0, .prev_dt_time = m_last_clock_check };

}

void bldc_perseus::set_target_position(hal::u16 target_position)
{
  m_target.position = target_position;
}

hal::u16 bldc_perseus::get_target_position()
{
  return m_target.position;
}

float bldc_perseus::get_current_position()
{

  m_current.position = m_encoder->read().angle;
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
  home_encoder_value = m_encoder->read().angle;
  m_current.position = 0;
}
// get_velocity ( previous position, curernt , dt) => ticks per ms
// MC sends set_velocity -> update_velocity() -> PID towards our target
// velocity
//
// need to add a dt component
// update_position() ->
// update_velocity() -> (different PIDs for when you do have feedforward and
// when you don't) PID separately for position and velocity at end of each
// function: m_h_bridge->power(output);

// velocity, no feedforward 
void bldc_perseus::update_velocity_noff() 
{
  // pid is pid-ing 
  // assuming in degrees/ms, need to multiply by 0.25 
  auto error = m_target.velocity - m_current.velocity;
  auto clock = resources::clock();
  auto console = resources::console();
  auto curr_time = clock->uptime();
  double dt = curr_time - m_PID_prev_velocity_values.prev_dt_time; 
  m_PID_prev_velocity_values.integral += error * dt; 
  auto derivative = (error - m_PID_prev_velocity_values.last_error) / dt; 
  auto pTerm = m_current_velocity_settings.kp * error; 
  auto iTerm  = m_current_velocity_settings.ki * m_PID_prev_velocity_values.integral; 
  auto dTerm = m_current_velocity_settings.kd * derivative; 
  m_PID_prev_velocity_values.last_error = error; 
  m_PID_prev_velocity_values.prev_dt_time = curr_time;


  // calculate velocity/ratio 
  // or if this is supposed to just return the PID values
  // edit to just return the summed terms 
  auto proj_vel = ((pTerm + iTerm + dTerm) * m_current.velocity); 
  hal::print<128>(*console, "P: %.6f, I: %.6f, D: %.6f\n", pTerm, iTerm, dTerm);
  hal::print<128>(*console, "Projected Velocity: %.6f\n", proj_vel);
  // return to h-bridge 
  // check if this is the right h_bridge 
  // m_h_bridge->power(proj_vel/360); 
}

// position, no feedforward 
void bldc_perseus::update_position_noff() 
{
  // pid is pid-ing 
  // assuming in degrees
  auto error = m_target.position - m_current.position; 
  auto clock = resources::clock(); 
  auto curr_time = clock->uptime();
  double dt = curr_time - m_PID_prev_position_values.prev_dt_time; 
  m_PID_prev_position_values.integral += error * dt; 
  auto derivative = (error - m_PID_prev_position_values.last_error) / dt; 
  auto pTerm = m_current_position_settings.kp * error; 
  auto iTerm  = m_current_position_settings.ki * m_PID_prev_position_values.integral; 
  auto dTerm = m_current_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_dt_time = curr_time; 
  
  // unsure of the next part for position, might increase velocity more than expected 
  // calculate velocity/ratio 
  // or if this is supposed to just return the PID values
  // edit to just return the summed terms 
  auto proj_pos = ((pTerm + iTerm + dTerm) * m_current.position); 

  // return to h-bridge 
  // check if this is the right h_bridge 
  m_h_bridge->power(proj_pos/360); 
}

// doing this straight from the encoder? 
void bldc_perseus::get_current_velocity() 
{
  auto terminal = resources::console();
  auto clock = resources::clock();

  float cv = m_encoder->read().angle; 
  hal::delay(*clock, 10ms);
  cv = cv - m_encoder->read().angle; 
  hal::print<32>(*terminal, "deg/ms: %.6f\n", cv);
}
}  // namespace sjsu::perseus
