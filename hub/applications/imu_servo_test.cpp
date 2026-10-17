#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <gimbal.hpp>
#include <icm20948_adapters.hpp>
#include <resource_list.hpp>

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

  auto gyro = resources::gyroscope();
  auto accel = resources::accelerometer();

  auto p_yaw_servo = resources::mast_servo_aboutx();
  auto p_pitch_servo = resources::mast_gimbal_abouty();
  auto p_angle_estimator = resources::imu_angle_estimator();

  int print_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    auto raw_accel = accel->read();
    auto raw_gyro = gyro->read();

    p_angle_estimator->update();
    auto aboutx = p_angle_estimator->get_angle_aboutx();
    p_pitch_servo->update(aboutx);

    print_count++;
    if (print_count >= 100) {
      hal::print<128>(*console,
                      "accel=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)\n",
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
