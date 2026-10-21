#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/gyroscope.hpp>

#include <gimbal.hpp>
#include <mission_control_manager.hpp>
#include <resource_list.hpp>

#include <icm20948_adapters.hpp>

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

  hal::print(*console, "init mission control manager\n");

  // auto p_mast_gimbal_controller =
  // resources::mast_gimbal_controller();

  auto mast_about_x = resources::mast_servo_about_x();
  auto mast_about_y = resources::mast_servo_about_y();
  auto wheels_about_x = resources::wheels_servo_about_x();
  drivers::clamped_servo clamped_mast_about_x{
    mast_about_x,
    {
      .min = 0,
      .max = 180,
    },
  };
  drivers::clamped_servo clamped_mast_about_y{
    mast_about_y,
    {
      .min = 0,
      .max = 180,
    },
  };
  drivers::clamped_servo clamped_wheels_about_x{
    wheels_about_x,
    {
      .min = 0,
      .max = 180,
    },
  };

  hal::print(*console, "init mast gimbal manager\n");

  // bool accel_on = false;
  // bool gyro_on = false;
  // bool mag_on = false;

  int send_count = 0;

  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  while (true) {
    // p_mast_gimbal_controller->update();
    // hal::print(*console, "frame\n");

    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      clamped_mast_about_x.position(gimbal_req->mast_angle_x);
      clamped_mast_about_y.position(gimbal_req->mast_angle_y);
      clamped_wheels_about_x.position(gimbal_req->wheels_angle_x);
      // p_mast_gimbal_controller->set_target(gimbal_req.value());

      hal::print<64>(*console,
                     "gimbal cmd: mast_x=%d mast_y=%d wheels_x=%d\n",
                     gimbal_req->mast_angle_x,
                     gimbal_req->mast_angle_y,
                     gimbal_req->wheels_angle_x);
    }

    // auto toggle_req = mcm.read_imu_toggle_request();
    // if (toggle_req) {
    //   accel_on = toggle_req->accel_on;
    //   gyro_on = toggle_req->gyro_on;
    //   mag_on = toggle_req->mag_on;
    //   hal::print<64>(
    //     *console, "imu toggle: a=%d g=%d m=%d\n", accel_on, gyro_on,
    //     mag_on);
    // }

    send_count++;
    if (send_count >= send_interval) {
      send_count = 0;

      // try {
      //   mcm.send_servo_position(
      //     p_mast->get_yaw_angle(), p_mast->get_pitch_angle(),
      //     wheels_yaw);
      // } catch (hal::resource_unavailable_try_again err) {
      //   hal::print(*console, "resource unavailable\n");
      // } catch (...) {
      //   hal::print(*console, "other error\n");
      // }

      // if (accel_on) {
      //   mcm.send_imu_accel(
      //     round_clamp_int16(raw_accel.x, raw_accel.y, raw_accel.z));
      // }
      // if (gyro_on) {
      //   mcm.send_imu_gyro(
      //     round_clamp_int16(raw_gyro.x, raw_gyro.y, raw_gyro.z));
      // }
      // if (mag_on) {
      //   mcm.send_imu_mag(round_clamp_int16(raw_mag.x, raw_mag.y,
      //   raw_mag.z));
      // }
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub
