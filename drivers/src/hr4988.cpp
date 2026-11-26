#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <hr4988.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

using namespace std::chrono_literals;

namespace sjsu::drivers {
hr4988::hr4988(hal::v5::strong_ptr<hal::output_pin> p_direction_pin,
               hal::v5::strong_ptr<hal::output_pin> p_step_pin,
               hal::v5::strong_ptr<hal::output_pin> p_enable_pin,
               hal::v5::strong_ptr<hal::steady_clock> p_steady_clock,
               step_factor p_motor_step_factor,
               int p_full_steps_per_rotation,
               hal::time_duration p_step_half_period,
               std::array<hal::v5::strong_ptr<hal::output_pin>, 3> p_mode_pins,
               hal::basic_servo::position_range_t p_position_range)
  : m_direction_pin(std::move(p_direction_pin))
  , m_step_pin(std::move(p_step_pin))
  , m_enable_pin(std::move(p_enable_pin))
  , m_mode_pins(std::move(p_mode_pins))
  , m_clock(std::move(p_steady_clock))
  , m_full_steps_per_rotation(p_full_steps_per_rotation)
  , m_step_half_period(p_step_half_period)
  , m_position_range(p_position_range)
{
  if (static_cast<std::size_t>(p_motor_step_factor) >=
        step_factor_codes.size() ||
      m_full_steps_per_rotation <= 0 || m_step_half_period < 1us ||
      !std::isfinite(m_position_range.min) ||
      !std::isfinite(m_position_range.max) || m_position_range.min > 0 ||
      m_position_range.max < 0) {
    throw hal::argument_out_of_domain(this);
  }

  // Don't drive the pins until the driver is ready.
  m_enable_pin->level(true);  // Active-low
  m_step_pin->level(false);
  m_direction_pin->level(false);

  set_step_factor(p_motor_step_factor);
  driver_enable(true);
}

// basic_servo interface:

void hr4988::driver_enable(bool p_state)
{
  m_enable_pin->level(!p_state);
  m_enabled = p_state;
}

void hr4988::driver_position(hal::degrees p_target_position)
{
  if (!m_enabled) {
    throw hal::resource_unavailable_try_again(this);
  }

  long const pulse_count = calculate_pulse_count(p_target_position);
  generate_pulses(pulse_count);
}

hal::basic_servo::position_range_t hr4988::driver_position_range()
{
  return m_position_range;
}

// Helper functions:

void hr4988::set_step_factor(step_factor p_step_factor)
{
  // Match the enum to the step_factor code.
  auto const index = static_cast<std::size_t>(p_step_factor);
  if (index >= step_factor_codes.size()) {
    throw hal::argument_out_of_domain(this);
  }
  hal::byte const step_factor_code = step_factor_codes.at(index);

  for (std::size_t i = 0; i < m_mode_pins.size(); ++i) {
    m_mode_pins[i]->level((step_factor_code & (1u << i)) != 0);
  }

  m_step_factor = p_step_factor;
}

long hr4988::calculate_pulse_count(hal::degrees p_target_position) const
{
  if (!std::isfinite(p_target_position) ||
      p_target_position < m_position_range.min ||
      p_target_position > m_position_range.max) {
    throw hal::argument_out_of_domain(this);
  }

  long const units_per_pulse = 128L >> static_cast<int>(m_step_factor);

  double const pulses_per_rotation =
    static_cast<double>(m_full_steps_per_rotation) * 128.0 /
    static_cast<double>(units_per_pulse);

  // Round to the nearest micro-step.
  double const target_pulses = std::round(
    static_cast<double>(p_target_position) / 360.0 * pulses_per_rotation);

  double const target_partial_steps =
    target_pulses * static_cast<double>(units_per_pulse);

  double const difference =
    std::round(target_pulses - static_cast<double>(m_partial_steps) /
                                 static_cast<double>(units_per_pulse));

  constexpr double min_long =
    static_cast<double>(std::numeric_limits<long>::min());
  constexpr double max_long_exclusive = -min_long;

  if (!std::isfinite(target_partial_steps) || target_partial_steps < min_long ||
      target_partial_steps >= max_long_exclusive ||
      !std::isfinite(difference) || difference < min_long ||
      difference >= max_long_exclusive) {
    throw hal::argument_out_of_domain(this);
  }

  return static_cast<long>(difference);
}

void hr4988::generate_pulses(long p_pulse_count)
{
  if (p_pulse_count == 0) {
    return;
  }

  long const units_per_pulse = 128L >> static_cast<int>(m_step_factor);
  bool const is_forward = p_pulse_count > 0;
  long const increment = is_forward ? units_per_pulse : -units_per_pulse;

  // Unsigned subtraction also handles the most-negative signed pulse count.
  unsigned long pulses = is_forward
                           ? static_cast<unsigned long>(p_pulse_count)
                           : 0UL - static_cast<unsigned long>(p_pulse_count);

  m_direction_pin->level(is_forward);

  hal::delay(*m_clock, 250ns);

  for (; pulses > 0; --pulses) {
    m_step_pin->level(true);
    m_partial_steps += increment;
    hal::delay(*m_clock, m_step_half_period);
    m_step_pin->level(false);
    hal::delay(*m_clock, m_step_half_period);
  }
}

}  // namespace sjsu::drivers
