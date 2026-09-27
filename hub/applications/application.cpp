#include <algorithm>
#include <cmath>
#include <cstdint>
#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <limits>
#include <optional>

#include <gimbal.hpp>
#include <mission_control_manager.hpp>
#include <resource_list.hpp>

#include <icm20948_adapters.hpp>

#include <math_helpers.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace {

constexpr int send_interval = 10;
}  // namespace

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "=== HUB APPLICATION START ===\n");

  hal::print(*console, "acquiring i2c...\n");
  auto i2c = resources::i2c();
  hal::print(*console, "i2c OK\n");

  hal::print(*console, "acquiring pwm channel 0...\n");
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  hal::print(*console, "pwm channel 0 OK\n");

  hal::print(*console, "acquiring pwm channel 1...\n");
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  hal::print(*console, "pwm channel 1 OK\n");

  hal::print(*console, "acquiring CAN transceiver...\n");
  auto can_transceiver = resources::can_transceiver();
  hal::print(*console, "CAN transceiver OK\n");

  hal::print(*console, "acquiring CAN bus manager...\n");
  auto can_bus_manager = resources::can_bus_manager();
  hal::print(*console, "CAN bus manager OK\n");

  hal::print(*console, "creating MCM...\n");
  mission_control_manager mcm(can_transceiver);
  hal::print(*console, "MCM OK\n");

  hal::print(*console, "creating & initializing ICM20948...\n");
  auto icm_device = resources::icm();
  hal::print(*console, "ICM20948 OK\n");

  hal::print(*console, "creating gyro source...\n");
  auto gyro = resources::gyroscope();
  hal::print(*console, "gyro source OK\n");

  hal::print(*console, "creating accel source...\n");
  auto accel = resources::accelerometer();
  hal::print(*console, "accel source OK\n");

  hal::print(*console, "creating mag source...\n");
  auto mag = resources::magnetometer();
  hal::print(*console, "mag source OK\n");

  hal::print(*console, "acquiring PWM frequency managers...\n");
  auto pwm_freq_tim1 = resources::pwm_frequency_tim1();
  auto pwm_freq_tim2 = resources::pwm_frequency_tim2();
  hal::print(*console, "PWM frequency managers OK\n");

  hal::print(*console, "creating yaw servo...\n");
  auto p_yaw_servo = resources::yaw_servo();
  hal::print(*console, "yaw servo OK\n");

  hal::print(*console, "creating pitch servo...\n");
  auto p_pitch_servo = resources::pitch_servo();
  hal::print(*console, "pitch servo OK\n");

  hal::print(*console, "creating gimbal...\n");
  auto p_mast = resources::mast();
  hal::print(*console, "gimbal OK\n");

  bool accel_on = false;
  bool gyro_on = false;
  bool mag_on = false;

  constexpr float dt = 0.01f;
  int send_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    hal::print(*console, "a\n");
    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      p_mast->set_target(gimbal_req->x_angle, gimbal_req->y_angle);
      hal::print<64>(*console,
                     "gimbal cmd: x=%d y=%d\n",
                     gimbal_req->x_angle,
                     gimbal_req->y_angle);
    }

    hal::print(*console, "b\n");
    auto toggle_req = mcm.read_imu_toggle_request();
    if (toggle_req) {
      accel_on = toggle_req->accel_on;
      gyro_on = toggle_req->gyro_on;
      mag_on = toggle_req->mag_on;
      hal::print<64>(
        *console, "imu toggle: a=%d g=%d m=%d\n", accel_on, gyro_on, mag_on);
    }

    hal::print(*console, "c\n");
    auto raw_accel = accel->read();

    hal::print(*console, "d\n");
    auto raw_gyro = gyro->read();

    hal::print(*console, "e\n");
    auto raw_mag = mag->read();

    hal::print(*console, "f\n");
    p_mast->update_pitch_servo(dt, raw_accel, raw_gyro);

    hal::print(*console, "g\n");

    send_count++;
    if (send_count >= send_interval) {
      send_count = 0;

      mcm.send_servo_position(p_mast->get_yaw_angle(), p_mast->get_pitch_angle());

      if (accel_on) {
        mcm.send_imu_accel(
          round_clamp_int16(raw_accel.x, raw_accel.y, raw_accel.z));
      }
      if (gyro_on) {
        mcm.send_imu_gyro(
          round_clamp_int16(raw_gyro.x, raw_gyro.y, raw_gyro.z));
      }
      if (mag_on) {
        mcm.send_imu_mag(round_clamp_int16(raw_mag.x, raw_mag.y, raw_mag.z));
      }
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub
