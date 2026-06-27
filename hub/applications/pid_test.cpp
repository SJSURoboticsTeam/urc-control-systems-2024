#include <algorithm>
#include <cmath>
#include <cstdint>
#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <limits>

#include <gimbal.hpp>
#include <resource_list.hpp>

#include <icm20948_sources.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace {
constexpr int min_pulse_width_range = 900;
constexpr int max_pulse_width_range = 2100;
}  // namespace

constexpr hal::actuator::rc_servo16::settings gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = min_pulse_width_range,
  .max_microseconds = max_pulse_width_range,
};

// IMU + servo test. No CAN required.
// Calibrates gyro at startup, then runs PID on pitch servo.
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "=== IMU + SERVO TEST (no CAN) ===\n");

  hal::print(*console, "acquiring i2c...\n");
  auto i2c = resources::i2c();
  hal::print(*console, "i2c OK\n");

  hal::print(*console, "creating ICM20948...\n");
  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c, *clock);
  hal::print(*console, "ICM20948 OK\n");

  hal::print(*console, "initializing magnetometer...\n");
  icm_device->init_mag();
  hal::print(*console, "magnetometer OK\n");

  icm_device->auto_offsets();

  hal::print(*console, "creating sensor sources...\n");
  auto gyro = hal::v5::make_strong_ptr<icm20948_gyro_source>(
    resources::driver_allocator(), icm_device);
  auto accel = hal::v5::make_strong_ptr<icm20948_accel_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "sensor sources OK\n");

  hal::print(*console, "acquiring PWM frequency managers...\n");
  auto pwm_freq_tim1 = resources::pwm_frequency_tim1();
  auto pwm_freq_tim2 = resources::pwm_frequency_tim2();
  hal::print(*console, "PWM frequency managers OK\n");

  hal::print(*console, "acquiring PWM channels...\n");
  auto pwm_ch0 = resources::mast_servo_pwm_channel_0();
  auto pwm_ch1 = resources::mast_servo_pwm_channel_1();
  hal::print(*console, "PWM channels OK\n");

  hal::print(*console, "creating X servo...\n");
  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    *pwm_freq_tim1,
    pwm_ch0,
    gimbal_servo_settings);
  hal::print(*console, "X servo OK\n");

  hal::print(*console, "creating Y servo...\n");
  auto p_y_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    *pwm_freq_tim2,
    pwm_ch1,
    gimbal_servo_settings);
  hal::print(*console, "Y servo OK\n");

  hal::print(*console, "creating gimbal...\n");
  gimbal mast(p_x_servo,
              p_y_servo,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);
  hal::print(*console, "gimbal OK\n");

  constexpr float dt = 0.01f;
  int print_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    auto raw_accel = accel->read_acceleration();
    auto raw_gyro = gyro->read_gyroscope();



    mast.update_y_servo(dt, raw_accel, raw_gyro);

    print_count++;
    if (print_count >= 100) {
      hal::print<128>(
        *console,
        "pos=(%d,%d) accel=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)\n",
        mast.get_x_angle(), mast.get_y_angle(),
        raw_accel.x, raw_accel.y, raw_accel.z,
        raw_gyro.x, raw_gyro.y, raw_gyro.z);
      print_count = 0;
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub