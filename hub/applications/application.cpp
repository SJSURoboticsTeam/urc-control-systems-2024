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
  
  auto can_transceiver = resources::can_transceiver();
  mission_control_manager mcm(can_transceiver);

  auto gyro = resources::gyroscope();
  auto accel = resources::accelerometer();
  auto mag = resources::magnetometer();

  auto p_mast = resources::mast();

  bool accel_on = false;
  bool gyro_on = false;
  bool mag_on = false;

  constexpr float dt = 0.01f;
  int send_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      p_mast->set_target(gimbal_req->x_angle, gimbal_req->y_angle);
      hal::print<64>(*console,
                     "gimbal cmd: x=%d y=%d\n",
                     gimbal_req->x_angle,
                     gimbal_req->y_angle);
    }

    auto toggle_req = mcm.read_imu_toggle_request();
    if (toggle_req) {
      accel_on = toggle_req->accel_on;
      gyro_on = toggle_req->gyro_on;
      mag_on = toggle_req->mag_on;
      hal::print<64>(
        *console, "imu toggle: a=%d g=%d m=%d\n", accel_on, gyro_on, mag_on);
    }

    auto raw_accel = accel->read();
    auto raw_gyro = gyro->read();
    auto raw_mag = mag->read();

    p_mast->update_pitch_servo(dt, raw_accel, raw_gyro);

    send_count++;
    if (send_count >= send_interval) {
      send_count = 0;

      mcm.send_servo_position(p_mast->get_yaw_angle(),
                              p_mast->get_pitch_angle());

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
