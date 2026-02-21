#include <algorithm>
#include <limits>
#include <cmath>
#include <cstdint>
#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <optional>
#include <sys/types.h>

#include <resource_list.hpp>
#include <gimbal.hpp>
#include <mission_control_manager.hpp>
namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

namespace{
  constexpr int min_pulse_width_range = 900;
  constexpr int max_pulse_width_range = 2100; 

  constexpr auto baudrate = 100.0_kHz;
  constexpr auto gimbal_read_id = 0x300;
  constexpr auto heartbeat_read_id = 0x0E;

  constexpr long int16_min = std::numeric_limits<std::int16_t>::min();
  constexpr long int16_max = std::numeric_limits<std::int16_t>::max();

  uint8_t imu_status = 0x0;
  uint8_t lcd_status = 0x0;
}

constexpr hal::actuator::rc_servo::settings gimbal_servo_settings{
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

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();
  
  mission_control_manager mcm(can_transceiver);

  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c);
  icm_device->init_mag();
  icm_device->auto_offsets();
  hal::print(*console, "icm initialized");

  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_0,
    gimbal_servo_settings);

  auto p_y_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_1,
    gimbal_servo_settings);

  hal::print(*console, "servo initialized");
  
  gimbal mast(p_x_servo,
              p_y_servo,
              icm_device,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);

  can_bus_manager->baud_rate(baudrate);
  can_id_filter->allow(gimbal_read_id);
  can_id_filter->allow(heartbeat_read_id);

  auto last_time = clock->uptime();
  hal::print(*console, "starting loop initialized");
  while (true) {
    auto now_time = clock->uptime();

    // Reset heartbeat status
    imu_status = 0x0;
    lcd_status = 0x0;

    // Determine the dt of the loop
    float dt = std::chrono::duration<float>(now_time - last_time).count();
    if (dt <= 0.0f)
      dt = 1e-6f;

    // Record the time as previous
    last_time = now_time;

    // Find incoming CAN msgs
    auto gimbal_req = mcm.read_gimbal_move_request();
    if (gimbal_req) {
    hal::print(*console, "gimbal_move_requested");
    switch (gimbal_req->direction) {
      case gimbal_direction::left:  mast.move_left(gimbal_req->offset);  break;
      case gimbal_direction::right: mast.move_right(gimbal_req->offset); break;
      case gimbal_direction::up:    mast.move_up(gimbal_req->offset);    break;
      case gimbal_direction::down:  mast.move_down(gimbal_req->offset);  break;
    }
  }
    // Data to send to MC
    auto raw_accel = icm_device->read_acceleration();
    auto raw_gyro = icm_device->read_gyroscope();
    auto raw_mag = icm_device->read_magnetometer();

    imu_status = 0x1;

    auto raw_accel_int16 =
      round_clamp_int16(raw_accel.x, raw_accel.y, raw_accel.z);
    auto raw_gyro_int16 = round_clamp_int16(raw_gyro.x, raw_gyro.y, raw_gyro.z);
    auto raw_mag_int16 = round_clamp_int16(raw_mag.x, raw_mag.y, raw_mag.z);

    bool hb = mcm.reply_heartbeat(imu_status, lcd_status);
    if(hb){
      mcm.reply_imu_accel_request(raw_accel_int16);
      mcm.reply_imu_gyro_request(raw_gyro_int16);
      mcm.reply_imu_mag_request(raw_mag_int16);
      mcm.clear_heartbeat_requests();
    }
    
    // Update the pitch servo
    mast.update_y_servo(dt);

    hal::delay(*clock, 2ms);
  }
}
}  // namespace sjsu::hub