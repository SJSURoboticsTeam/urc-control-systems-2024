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

#include <icm20948_adapters.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

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
  auto icm_device = resources::icm();
  hal::print(*console, "ICM20948 OK\n");

  hal::print(*console, "initializing magnetometer...\n");
  icm_device->init_mag();
  hal::print(*console, "magnetometer OK\n");

  icm_device->auto_offsets();

  hal::print(*console, "creating sensor sources...\n");
  auto gyro = resources::gyroscope();
  auto accel = resources::accelerometer();
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
  auto p_yaw_servo = resources::yaw_servo();
  hal::print(*console, "X servo OK\n");

  hal::print(*console, "creating Y servo...\n");
  auto p_pitch_servo = resources::pitch_servo();
  hal::print(*console, "Y servo OK\n");

  hal::print(*console, "creating gimbal...\n");
  auto p_mast = resources::mast();
  hal::print(*console, "gimbal OK\n");

  constexpr float dt = 0.01f;
  int print_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    auto raw_accel = accel->read();
    auto raw_gyro = gyro->read();

    p_mast->update_pitch_servo(dt, raw_accel, raw_gyro);

    print_count++;
    if (print_count >= 100) {
      hal::print<128>(
        *console,
        "pos=(%d,%d) accel=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)\n",
        p_mast->get_yaw_angle(),
        p_mast->pitch(),
        raw_accel.x,
        raw_accel.y,
        raw_accel.z,
        raw_gyro.x,
        raw_gyro.y,
        raw_gyro.z);
      print_count = 0;
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub
