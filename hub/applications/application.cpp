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

#include <icm20948_sources.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace {
constexpr int min_pulse_width_range = 900;
constexpr int max_pulse_width_range = 2100;

constexpr long int16_min = std::numeric_limits<std::int16_t>::min();
constexpr long int16_max = std::numeric_limits<std::int16_t>::max();

// ~10Hz: send every 10th loop iteration (10ms * 10 = 100ms)
constexpr int send_interval = 10;
}  // namespace

constexpr hal::actuator::rc_servo16::settings gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = min_pulse_width_range,
  .max_microseconds = max_pulse_width_range,
};

int16_axis round_clamp_int16(float init_x, float init_y, float init_z)
{
  long const x_long = lroundf(init_x);
  long const y_long = lroundf(init_y);
  long const z_long = lroundf(init_z);

  return int16_axis{
    .x = static_cast<int16_t>(std::clamp<long>(x_long, int16_min, int16_max)),
    .y = static_cast<int16_t>(std::clamp<long>(y_long, int16_min, int16_max)),
    .z = static_cast<int16_t>(std::clamp<long>(z_long, int16_min, int16_max))
  };
}

// Production hub application with servos, PID, CAN, and IMU.
// For CAN-only testing without servos, use can_test.cpp instead.
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();

  auto i2c = resources::i2c();
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();

  mission_control_manager mcm(can_transceiver);

  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c, *clock);
  icm_device->init_mag();

  auto gyro = hal::v5::make_strong_ptr<icm20948_gyro_source>(
    resources::driver_allocator(), icm_device);
  auto accel = hal::v5::make_strong_ptr<icm20948_accel_source>(
    resources::driver_allocator(), icm_device);
  auto mag = hal::v5::make_strong_ptr<icm20948_mag_source>(
    resources::driver_allocator(), icm_device);

  auto pwm_freq_tim1 = resources::pwm_frequency_tim1();
  auto pwm_freq_tim2 = resources::pwm_frequency_tim2();

  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    *pwm_freq_tim1,
    mast_servo_pwm_channel_0,
    gimbal_servo_settings);

  auto p_y_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo16>(
    resources::driver_allocator(),
    *pwm_freq_tim2,
    mast_servo_pwm_channel_1,
    gimbal_servo_settings);

  gimbal mast(p_x_servo,
              p_y_servo,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);

  // IMU stream toggle state (all off by default)
  bool accel_on = false;
  bool gyro_on = false;
  bool mag_on = false;

  constexpr float dt = 0.01f;
  int send_count = 0;

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    // Process gimbal target command (0x300)
    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      mast.set_target(gimbal_req->x_angle, gimbal_req->y_angle);
    }

    // Process IMU toggle command (0x305)
    auto toggle_req = mcm.read_imu_toggle_request();
    if (toggle_req) {
      accel_on = toggle_req->accel_on;
      gyro_on = toggle_req->gyro_on;
      mag_on = toggle_req->mag_on;
    }

    // Read sensors and update PID
    auto raw_accel = accel->read_acceleration();
    auto raw_gyro = gyro->read_gyroscope();
    auto raw_mag = mag->read_magnetometer();

    mast.update_y_servo(dt, raw_accel, raw_gyro);

    // Periodic CAN sends at ~10Hz
    send_count++;
    if (send_count >= send_interval) {
      send_count = 0;

      // Servo position acts as heartbeat (0x306)
      mcm.send_servo_position(mast.get_x_angle(), mast.get_y_angle());

      // IMU data only when toggled on via 0x305
      if (accel_on) {
        mcm.send_imu_accel(
          round_clamp_int16(raw_accel.x, raw_accel.y, raw_accel.z));
      }
      if (gyro_on) {
        mcm.send_imu_gyro(
          round_clamp_int16(raw_gyro.x, raw_gyro.y, raw_gyro.z));
      }
      if (mag_on) {
        mcm.send_imu_mag(
          round_clamp_int16(raw_mag.x, raw_mag.y, raw_mag.z));
      }
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub