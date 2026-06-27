#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <resource_list.hpp>

namespace sjsu::hub {

using namespace std::chrono_literals;
using namespace hal::literals;

namespace {
constexpr int min_pulse_width_range = 900;
constexpr int max_pulse_width_range = 2100;
}  // namespace

constexpr hal::actuator::rc_servo16::settings servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = min_pulse_width_range,
  .max_microseconds = max_pulse_width_range,
};

// Simple servo sweep test. No CAN, no IMU, no PID.
// If servos don't move here, it's a hardware/wiring issue.
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "=== SERVO SWEEP TEST ===\n");

  hal::print(*console, "acquiring pwm channel 0 (PA8)...\n");
  auto pwm_ch0 = resources::mast_servo_pwm_channel_0();
  hal::print(*console, "pwm channel 0 OK\n");

  hal::print(*console, "acquiring pwm channel 1 (PA1)...\n");
  auto pwm_ch1 = resources::mast_servo_pwm_channel_1();
  hal::print(*console, "pwm channel 1 OK\n");

  hal::print(*console, "acquiring PWM frequency managers...\n");
  auto pwm_freq_tim1 = resources::pwm_frequency_tim1();
  auto pwm_freq_tim2 = resources::pwm_frequency_tim2();
  hal::print(*console, "PWM frequency managers OK\n");

  hal::print(*console, "creating X servo...\n");
  hal::actuator::rc_servo16 x_servo(*pwm_freq_tim1, pwm_ch0, servo_settings);
  hal::print(*console, "X servo OK\n");

  hal::print(*console, "creating Y servo...\n");
  hal::actuator::rc_servo16 y_servo(*pwm_freq_tim2, pwm_ch1, servo_settings);
  hal::print(*console, "Y servo OK\n");

  hal::print(*console, "starting sweep in 2 seconds...\n");
  hal::delay(*clock, 2000ms);

  hal::print(*console, "=== SWEEPING ===\n");
  while (true) {
    hal::print(*console, "X -> 0, Y -> 30 (both min)\n");
    x_servo.position(0.0f);
    y_servo.position(30.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "X -> 90, Y -> 90 (both center)\n");
    x_servo.position(90.0f);
    y_servo.position(90.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "X -> 180, Y -> 150 (both max)\n");
    x_servo.position(180.0f);
    y_servo.position(150.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "X -> 90, Y -> 90 (both center)\n");
    x_servo.position(90.0f);
    y_servo.position(90.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "--- loop complete, repeating ---\n");
  }
}
}  // namespace sjsu::hub
