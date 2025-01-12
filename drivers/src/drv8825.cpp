#include <drv8825.hpp>
#include <libhal/output_pin.hpp>

using namespace std::chrono_literals;

namespace sjsu::drivers {
drv8825::drv8825(ctor_params const& p_params)
  : m_direction_pin(p_params.direction_pin)
  , m_step_pin(p_params.step_pin)
  , m_clock(p_params.steady_clock)
  , m_step_half_period(p_params.step_half_period)
  , m_mode_pins(p_params.mode_pins)
  , m_full_steps_per_rotation(p_params.full_steps_per_rotation)
{
  set_step_factor(p_params.motor_step_factor);
}

void drv8825::step(long p_steps)
{
  m_partial_steps += p_steps * (32 >> static_cast<int>(m_step_factor));
  if (p_steps < 0) {
    p_steps *= -1;
    m_direction_pin.level(false);
  } else {
    m_direction_pin.level(true);
  }
  // 650ns delay for direction change on datasheet (extra 50ns for tolerance)
  hal::delay(m_clock, 700ns);
  for (; p_steps > 0; p_steps--) {
    m_step_pin.level(true);
    hal::delay(m_clock, m_step_half_period);
    m_step_pin.level(false);
    hal::delay(m_clock, m_step_half_period);
  }
}

void drv8825::set_step_factor(step_factor p_step_factor)
{
  m_step_factor = p_step_factor;
  hal::byte mode = static_cast<hal::byte>(m_step_factor);
  if (mode & 0b001) {
    m_mode_pins[0]->level(true);
  } else {
    m_mode_pins[0]->level(false);
  }
  if (mode & 0b010) {
    m_mode_pins[1]->level(true);
  } else {
    m_mode_pins[1]->level(false);
  }
  if (mode & 0b100) {
    m_mode_pins[2]->level(true);
  } else {
    m_mode_pins[2]->level(false);
  }
}

long drv8825::get_partial_steps()
{
  return m_partial_steps;
}

hal::degrees drv8825::get_position()
{
  float full_steps = static_cast<float>(m_partial_steps) / 32.0f;
  float rotations = full_steps * static_cast<float>(m_full_steps_per_rotation);
  float degrees = rotations * 360.0f;
  return degrees;
}

void drv8825::driver_position(hal::degrees p_position)
{
  float rotations = p_position / 360.f;
  float full_steps = rotations * static_cast<float>(m_full_steps_per_rotation);
  float target_partial_steps = full_steps * 32.0f;
  long difference = target_partial_steps - static_cast<long>(m_partial_steps);
  step(difference);
}
};  // namespace sjsu::drivers
