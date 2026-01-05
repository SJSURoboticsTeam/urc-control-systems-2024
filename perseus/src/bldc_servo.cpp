#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

#include <bldc_servo.hpp>

#include "../resource_list.hpp"

using namespace std::chrono_literals;

namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : m_h_bridge(p_hbridge)
  , m_encoder(p_encoder)
{
  auto clock = resources::clock(); 
  m_last_clock_check = clock->uptime(); 

  m_current = {
    .position = 0,
    .power = 0.1f, 
    .velocity = 0,  
  };
  m_target = { .position = 0, .power = 0.0f , .velocity = 0};
  m_clamped_speed = 0.3;
  m_prev_encoder_value = m_encoder->read().angle;
  m_PID_prev_velocity_values = {.integral = 0, .last_error = 0, .prev_dt_time = 0.0 };
  m_PID_prev_position_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
}

void bldc_perseus::set_target_position(float target_position)
{
  m_target.position = target_position;
}

float bldc_perseus::get_target_position()
{
  return m_target.position;
}

float bldc_perseus::get_current_position()
{
  m_current.position = m_encoder->read().angle;
  return m_current.position;
}

void bldc_perseus::set_target_velocity(float target_velocity)
{
  m_target.velocity = target_velocity;
}

float bldc_perseus::get_current_velocity_in_tps()
{
  // TODO!
  return m_current.velocity;
}

float bldc_perseus::get_current_velocity_percentage()
{
  // TODO!
  return m_current.power;
}
void bldc_perseus::set_current_velocity(float current_velocity)
{
  m_current.velocity = current_velocity;
}
float bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}

void bldc_perseus::stop()
{
  m_current.power = 0.0f;
  m_h_bridge->power(0.0f);
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
  // TODO!
  home_encoder_value = m_encoder->read().angle;
  m_current.position = 0;
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
  m_clamped_speed = power; 
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
  m_current.position = m_encoder->read().angle;
  float error = m_target.position - m_current.position;
  auto clock = resources::clock();
  sec curr_time = hal_time_duration_to_sec(get_clock_time(*clock));
  sec dt = curr_time - m_PID_prev_position_values.prev_dt_time;
  m_PID_prev_position_values.integral += error * dt; 
  float derivative = (error - m_PID_prev_position_values.last_error) / dt; // this turns out to be negative because hopefully your current error is less than your last error
  float pTerm = m_current_position_settings.kp * error; 
  float iTerm  = m_current_position_settings.ki * m_PID_prev_position_values.integral; 
  float dTerm = m_current_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_dt_time = curr_time;

  float proj_pos = pTerm + iTerm + dTerm;
  float ff_clamp =0.2;
  float ff = bldc_perseus::position_feedforward() * ff_clamp; // print this? maybe consider as another csv print
  
  float proj_power = std::clamp(proj_pos, -1 * m_clamped_speed, m_clamped_speed);

  m_current.power = proj_power; 
  print_csv_format(pTerm, iTerm, dTerm, proj_power, ff);
  m_h_bridge->power(m_current.power);
}

float bldc_perseus::position_feedforward() 
{
  float length = 0.5715;  // elbow_bar=19in=48.26cm=0.4826m
                          // shoulder_bar=22.5in=57.15cm=0.5715m
                          // wrist_bar=12in=30.48cm=0.3048m + 14in=76.2cm=0.762m
  float angle_offset = 0; // elbow=-20
                          // shoulder=0
                          // wrist=0
  float weight_beam = 1600 * 9.8; // elbow=1000g
                                  // shoulder=1600g
                                  // wrist=600g
  float weight_end = 1600 * 9.8;  // add together other parts
  
  float y_force = std::sin(std::numbers::pi/180 * (m_target.position + angle_offset)) 
      * length * (weight_beam/2 + weight_end);
  // for elbow 
  y_force = -1 * y_force / (length * (weight_beam/2 + weight_end));
  // // for wrist (might change)
  // if (m_current.position >= 0) y_force = -1 * y_force / (length * (weight_beam/2 + weight_end));  // >= only for elbow, > for others 
  // else if (m_current.position < 0) y_force =  y_force / (length * (weight_beam/2 + weight_end)); 
  return y_force; 
}

void bldc_perseus::print_csv_format(float pTerm, float iTerm, float dTerm, float proj_power, float ff)
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
} 


}// namespace sjsu::perseus
