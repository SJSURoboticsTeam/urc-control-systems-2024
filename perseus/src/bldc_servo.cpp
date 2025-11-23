#include "../include/bldc_servo.hpp"
#include <cstdio>
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include "../hardware_map.hpp"
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>
using namespace std::chrono_literals;
using sec = float;

namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : m_h_bridge(p_hbridge)
  , m_encoder(p_encoder)
{
  m_current = {
    .position = 0,
    .velocity = 0,  
    .power = 0.1f, // 
  };
  m_target = { .position = 0, .velocity = 0, .power = 0.0f };
  m_clamped_speed =
    0.3;  // 40*0.3 = 12V which is the maximum this servo can be drivern
  m_clamped_accel = 0.1;  // if we are currently at a velocity of +0.2, we must
                          // not immediately change our current velocity to
                          // -0.2, even if our target velocity changes to -0.2
  m_prev_encoder_value = m_encoder->read().angle;
  // inital/prev pid values 
  auto clock = resources::clock(); 
  m_last_clock_check = clock->uptime(); 
  m_PID_prev_velocity_values = {.integral = 0, .last_error = 0, .prev_dt_time = 0.0 };
  m_PID_prev_position_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
}

void bldc_perseus::set_target_position(float target_position)
{
  m_target.position = target_position;
  // update the maximum possible power we nee
  
  
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
  return m_current.power;
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
bldc_perseus::PID_settings bldc_perseus::get_pid_settings()
{
  return m_current_position_settings;
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

// ff clamped from -0.1 + 0.1
// pid power = -0.2 ->
// pid power other direction = 0.2 -0.1 = 0.1

// final answer also clamped from -0.3 to 0.3
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
  // m_h_bridge->power(proj_vel +  ff);
}

constexpr hal::time_duration sec_to_hal_time_duration(sec p_time)
{
  return static_cast<hal::time_duration>(static_cast<long long>(p_time * 1e9f));
}

constexpr sec hal_time_duration_to_sec(hal::time_duration p_time)
{
  return static_cast<float>(p_time.count()) * 1e-9f;
}
void bldc_perseus::set_max_power(float power)
{
  m_clamped_speed = power; 
}
hal::time_duration get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sjsu::perseus::sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}
// position, no feedforward 
void bldc_perseus::update_position_noff() 
{
  // pid is pid-ing
  // assuming in degrees
  m_current.position = m_encoder->read().angle;
  auto error = m_target.position - m_current.position;
  auto clock = resources::clock();
  auto console = resources::console();
  auto curr_time = hal_time_duration_to_sec(get_clock_time(*clock));
  sec dt = curr_time - m_PID_prev_position_values.prev_dt_time;
  // hal::print<128>(*console, "DT: %.6f\n", dt);
  // float k_step = 1;
  m_PID_prev_position_values.integral += error * dt; 
  auto derivative = (error - m_PID_prev_position_values.last_error) / dt; // this turns out to be negative because hopefully your current error is less than your last error
  auto pTerm = m_current_position_settings.kp * error; 
  auto iTerm  = m_current_position_settings.ki * m_PID_prev_position_values.integral; 
  auto dTerm = m_current_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_dt_time = curr_time;

  // unsure of the next part for position, might increase velocity more than
  // expected calculate velocity/ratio or if this is supposed to just return the
  // PID values edit to just return the summed terms
  auto proj_pos = pTerm + iTerm + dTerm;
  auto proj_power = std::clamp(proj_pos, -1 * m_clamped_speed, m_clamped_speed);
  float ff_clamp =0.2;
  auto ff = bldc_perseus::position_feedforward() * ff_clamp; // print this? maybe consider as another csv print
  // proj_pos = proj_pos; + ff;
  
  m_current.power = proj_power; // + ff;
  // return to h-bridge
  // check if this is the right h_bridge
  print_csv_format(pTerm, iTerm, dTerm, proj_power, ff);
  m_h_bridge->power(m_current.power);
}
// feedforward fun 
// called by update_position 
// return float? other value? 
// factors to consider: servo, direction, angle, additional weight
float sjsu::perseus::bldc_perseus::position_feedforward() 
{
  float pi = 3.14159; // should be M_PI in <cmath>?
  // the next three variables will all have to be measured/calculated for each servo 
  float length = 0.5715 ; // elbow_bar=19in=48.26cm=0.4826m
                          // shoulder_bar=22.5in=57.15cm=0.5715m
                          // wrist_bar=12in=30.48cm=0.3048m + 14in=76.2cm=0.762m
  float angle_offset = 0; // = measure monday 
  float weight_beam = 1600 * 9.8;  // measure  
  float weight_end = 1600 * 9.8;   // measure  
  
  // assue elbow 
  float y_force = std::sin(pi/180 * (m_target.position + angle_offset))  // need to convert degree to rad
      * length * (weight_beam/2 + weight_end);
  // y_force = y_force / (length * (weight_beam / 2 + weight_end));

  if (m_current.position >= 0) y_force = -1 * y_force / (length * (weight_beam/2 + weight_end));  // >= only for elbow, > for others 
  else if (m_current.position < 0) y_force =  y_force / (length * (weight_beam/2 + weight_end)); 
  // else {
  //   // homing would be velocity feed forward?? 
  // }
  return y_force; 
  // return std::clamp(y_force, static_cast<float>(-0.1), static_cast<float>(0.1)); 
}

void sjsu::perseus::bldc_perseus::print_csv_format(float pTerm, float iTerm, float dTerm, float proj_power, float ff)
{
  auto console = resources::console();
  hal::print<256>(
    *console,
    "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
    pTerm,
    iTerm,
    dTerm,
    proj_power,
    m_current.power,
    m_current.position,
    m_target.position,
    m_current_position_settings.kp,
    m_current_position_settings.ki,
    m_current_position_settings.kd,
    ff);
};
    

// doing this straight from the encoder? 
void bldc_perseus::get_current_velocity() 
{
  auto terminal = resources::console();
  auto clock = resources::clock();

  float cv = m_encoder->read().angle; 
  hal::delay(*clock, 10ms);
  cv = cv - m_encoder->read().angle; 
  hal::print<32>(*terminal, "deg/ms: %.6f\n", cv);
}// namespace sjsu::perseus

}