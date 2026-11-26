#pragma once

#include <array>
#include <cstdint>

#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

// cspell:ignore datasheet GPIO microstep microsteps
namespace sjsu::drivers {
/**
 * @brief Stepper motor driver for an HR4988-controlled servo.
 *
 * Implements hal::basic_servo:
 * - enable() controls the active-low motor output enable pin.
 * - position() moves to an absolute motor-shaft angle in degrees.
 * - position_range() returns the configured minimum and maximum angles.
 *
 * Microstep resolution is selected during construction.
 * The starting shaft position is considered as zero.
 */
class hr4988 : public hal::basic_servo
{
public:
  /**
   * @brief Number of microsteps per full motor step.
   */
  enum class step_factor : std::uint8_t
  {
    one = 0,
    one_over_2,
    one_over_4,
    one_over_8,
    one_over_16,
    one_over_32,
    one_over_64,
    one_over_128,
  };

  /**
   * @brief Initializes the driver and microstep resolution.
   *
   * @param p_direction_pin GPIO connected to DIR.
   * @param p_step_pin GPIO connected to STEP.
   * @param p_enable_pin GPIO connected to active-low ENABLE.
   * @param p_steady_clock Clock used for pulse timing.
   * @param p_motor_step_factor Microstep resolution.
   * @param p_full_steps_per_rotation Full steps per motor-shaft revolution.
   * @param p_step_half_period Duration of each STEP HIGH and LOW interval
   * (minimum 1us per datasheet).
   * @param p_mode_pins Mode pins ordered MS1, MS2, MS3.
   * @param p_position_range Range of motor-shaft angles in degrees (must
   * include 0).
   * @throws hal::argument_out_of_domain If the configuration is invalid.
   */
  explicit hr4988(
    hal::v5::strong_ptr<hal::output_pin> p_direction_pin,
    hal::v5::strong_ptr<hal::output_pin> p_step_pin,
    hal::v5::strong_ptr<hal::output_pin> p_enable_pin,
    hal::v5::strong_ptr<hal::steady_clock> p_steady_clock,
    step_factor p_motor_step_factor,
    int p_full_steps_per_rotation,
    hal::time_duration p_step_half_period,
    std::array<hal::v5::strong_ptr<hal::output_pin>, 3> p_mode_pins,
    hal::basic_servo::position_range_t p_position_range);

private:
  // Microstep lookup table from datasheet.
  // MSB is MS3.
  static constexpr std::array<hal::byte, 8> step_factor_codes = {
    0b000,  // 1
    0b001,  // 1/2
    0b010,  // 1/4
    0b011,  // 1/8
    0b111,  // 1/16
    0b101,  // 1/32
    0b110,  // 1/64
    0b100,  // 1/128
  };

  // basic_servo interface.
  void driver_enable(bool p_state) override;
  void driver_position(hal::degrees p_target_position) override;
  hal::basic_servo::position_range_t driver_position_range() override;

  // Helper functions
  void set_step_factor(step_factor p_step_factor);

  [[nodiscard]] long calculate_pulse_count(
    hal::degrees p_target_position) const;
  void generate_pulses(long p_pulse_count);

  // Pins
  hal::v5::strong_ptr<hal::output_pin> m_direction_pin;
  hal::v5::strong_ptr<hal::output_pin> m_step_pin;
  hal::v5::strong_ptr<hal::output_pin> m_enable_pin;
  std::array<hal::v5::strong_ptr<hal::output_pin>, 3> m_mode_pins;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;

  // Motor configuration.
  int const m_full_steps_per_rotation = 1;
  step_factor m_step_factor = step_factor::one;
  hal::time_duration m_step_half_period{};
  hal::basic_servo::position_range_t m_position_range{};

  // Signed position in 1/128 units.
  long m_partial_steps = 0;

  bool m_enabled = false;
};

}  // namespace sjsu::drivers
