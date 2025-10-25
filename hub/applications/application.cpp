#include "../hardware_map.hpp"
#include "../include/gimbal.hpp"
#include <chrono>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

// Pulse width range for FT5325M
#define MIN_PULSEWIDTH_RANGE 900
#define MAX_PULSEWIDTH_RANGE 2100
static hal::time_duration const wd_countdown_timer =
  std::chrono::nanoseconds(5000000000);  // 5 secs
// static hal::time_duration const wd_countdown_timer = <recovery - expr>(5);

namespace sjsu::hub {

hal::actuator::rc_servo::settings const gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = MIN_PULSEWIDTH_RANGE,
  .max_microseconds = MAX_PULSEWIDTH_RANGE,
};

void application()
{
  using namespace std::chrono_literals;
  auto clock = resources::clock();
  auto console = resources::console();
  //   auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  //   auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  //   auto i2c = resources::i2c();

  // Initializing ICM
  hal::sensor::icm20948 icm(*i2c);

  hal::print(*console, "test1\n");

  // List of watchdog commands to use
  hal::stm32f1::independent_watchdog hub_watchdog{};
  hub_watchdog.set_countdown_time(wd_countdown_timer);
  // hub_watchdog.start(); // Start watchdog countdown
  // hub_watchdog.reset();  // Reset watchdog countdown
  // hub_watchdog.check_flag(); // True if it reaches countdown; False if its
  // still during the countdown
  // hub_watchdog.clear_flag();  // Clears the flag

  // Initializing the servos for the gimbal (takes in pwm not pwm16_channel)
  // Ensure the connections to servo are correct
  //   hal::actuator::rc_servo p_x_servo(*mast_servo_pwm_channel_0,
  //                                     gimbal_servo_settings);
  //   hal::actuator::rc_servo p_y_servo(*mast_servo_pwm_channel_1,
  //                                     gimbal_servo_settings);

  // Initializing Watchdog

  //   hal::print(*console, "Starting Application!\n");
  //   hal::print(*console, "Will reset after ~10 seconds\n");

  //   for (int i = 0; i < 10; i++) {
  //     // Print message
  //     hal::print(*console, "Hello, World\n");
  //   }

  //   hal::print(*console, "Resetting!\n");
  //   hal::delay(*clock, 100ms);
  //   resources::reset();
}
}  // namespace sjsu::hub