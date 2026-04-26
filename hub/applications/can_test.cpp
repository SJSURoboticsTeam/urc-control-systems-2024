#include <algorithm>
#include <cmath>
#include <cstdint>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <limits>
#include <optional>

#include <mission_control_manager.hpp>
#include <resource_list.hpp>

#include <icm20948_sources.hpp>

namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace {
constexpr long int16_min = std::numeric_limits<std::int16_t>::min();
constexpr long int16_max = std::numeric_limits<std::int16_t>::max();

// ~10Hz: send every 10th loop iteration (10ms * 10 = 100ms)
constexpr int send_interval = 10;
}  // namespace

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

// This test verifies the full CAN + IMU pipeline without servos.
// Tests: receiving gimbal commands (0x300), IMU toggle (0x305),
// sending servo position heartbeat (0x306), and IMU data (0x301-0x303).
// Servos are not initialized -- servo position is tracked in software only.
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "=== HUB CAN TEST (no servos) ===\n");

  hal::print(*console, "acquiring i2c...\n");
  auto i2c = resources::i2c();
  hal::print(*console, "i2c OK\n");

  hal::print(*console, "acquiring CAN transceiver...\n");
  auto can_transceiver = resources::can_transceiver();
  hal::print(*console, "CAN transceiver OK\n");

  hal::print(*console, "acquiring CAN bus manager...\n");
  auto can_bus_manager = resources::can_bus_manager();
  hal::print(*console, "CAN bus manager OK\n");

  hal::print(*console, "creating mission control manager...\n");
  mission_control_manager mcm(can_transceiver);
  hal::print(*console, "MCM OK\n");

  hal::print(*console, "creating ICM20948...\n");
  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c, *clock);
  hal::print(*console, "ICM20948 OK\n");

  hal::print(*console, "initializing magnetometer...\n");
  icm_device->init_mag();
  hal::print(*console, "magnetometer OK\n");

  hal::print(*console, "creating gyro source...\n");
  auto gyro = hal::v5::make_strong_ptr<icm20948_gyro_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "gyro source OK\n");

  hal::print(*console, "creating accel source...\n");
  auto accel = hal::v5::make_strong_ptr<icm20948_accel_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "accel source OK\n");

  hal::print(*console, "creating mag source...\n");
  auto mag = hal::v5::make_strong_ptr<icm20948_mag_source>(
    resources::driver_allocator(), icm_device);
  hal::print(*console, "mag source OK\n");

  hal::print(*console, "NOTE: servos disabled (no hardware)\n");
  hal::print(*console, "=== ENTERING MAIN LOOP ===\n");

  // Simulated servo position, starts centered
  // Updates when 0x300 is received, sent back on 0x306 as heartbeat
  uint8_t x_angle = 90;
  uint8_t y_angle = 90;

  // IMU stream toggle state, all off by default
  // Updated when 0x305 is received
  bool accel_on = false;
  bool gyro_on = false;
  bool mag_on = false;

  int send_count = 0;
  int print_count = 0;

  while (true) {
    hal::u64 frame_end = hal::future_deadline(*clock, 10ms);

    // Check for gimbal target command (0x300)
    // Updates tracked servo angles, no actual servo movement
    auto gimbal_req = mcm.read_gimbal_target_request();
    if (gimbal_req) {
      x_angle = gimbal_req->x_angle;
      y_angle = gimbal_req->y_angle;
      hal::print<64>(*console,
                     "gimbal cmd: x=%d y=%d (servos disabled)\n",
                     x_angle, y_angle);
    }

    // Check for IMU toggle command (0x305)
    // Controls which IMU streams are sent on 0x301-0x303
    auto toggle_req = mcm.read_imu_toggle_request();
    if (toggle_req) {
      accel_on = toggle_req->accel_on;
      gyro_on = toggle_req->gyro_on;
      mag_on = toggle_req->mag_on;
      hal::print<64>(*console,
                     "imu toggle: accel=%d gyro=%d mag=%d\n",
                     accel_on, gyro_on, mag_on);
    }

    // Always read sensors, only send when toggled on
    auto raw_accel = accel->read_acceleration();
    auto raw_gyro = gyro->read_gyroscope();
    auto raw_mag = mag->read_magnetometer();

    // Periodic sends at ~10Hz (every 100ms)
    send_count++;
    if (send_count >= send_interval) {
      send_count = 0;

      // Servo position acts as heartbeat for MC (0x306)
      mcm.send_servo_position(x_angle, y_angle);

      // IMU data only sent when toggled on via 0x305
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

    // Debug print every ~1 second
    print_count++;
    if (print_count >= 100) {
      hal::print<128>(
        *console,
        "pos=(%d,%d) imu=[%d,%d,%d] accel=(%.2f,%.2f,%.2f)\n",
        x_angle, y_angle,
        accel_on, gyro_on, mag_on,
        raw_accel.x, raw_accel.y, raw_accel.z);
      print_count = 0;
    }

    while (clock->uptime() < frame_end)
      ;
  }
}
}  // namespace sjsu::hub