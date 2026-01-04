#include <algorithm>
#include <array>
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
#include "../include/gimbal.hpp"

// Pulse width range for FT5325M
#define MIN_PULSEWIDTH_RANGE 900
#define MAX_PULSEWIDTH_RANGE 2100

// static hal::time_duration const wd_countdown_timer = <recovery - expr>(5);
namespace sjsu::hub {

using namespace hal::literals;
using namespace std::chrono_literals;

// Watchdog Countdown Time
static constexpr auto wait_time = 5s;

// CAN Baudrate
static constexpr auto baudrate = 100.0_kHz;

// CAN IDs we want to read from
static constexpr auto gimbal_read_id = 0x300;
static constexpr auto heartbeat_read_id = 0x0e;

// CAN IDs we want to use to send to MC
static constexpr auto heartbeat_reply_id = 0x0f;
static constexpr auto imu_accel_reply_id = 0x301;
static constexpr auto imu_gyro_reply_id = 0x302;
static constexpr auto imu_mag_reply_id = 0x303;

// Heartbeat status
static uint8_t imu_status = 0x0;
static uint8_t lcd_status = 0x0;

// Initializing the servo settings for the gimbal
hal::actuator::rc_servo::settings const gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = MIN_PULSEWIDTH_RANGE,
  .max_microseconds = MAX_PULSEWIDTH_RANGE,
};

struct int16_t_axis
{
  int16_t x;
  int16_t y;
  int16_t z;
};

int16_t_axis round_clamp_int16(float init_x, float init_y, float init_z)
{
  long const x_long = lroundf(init_x);
  long const y_long = lroundf(init_y);
  long const z_long = lroundf(init_z);

  return int16_t_axis{
    .x = static_cast<int16_t>(std::clamp<long>(x_long, -32768L, 32767L)),
    .y = static_cast<int16_t>(std::clamp<long>(y_long, -32768L, 32767L)),
    .z = static_cast<int16_t>(std::clamp<long>(z_long, -32768L, 32767L))
  };
}

void handle_gimbal_can_command(gimbal& gimbal, hal::can_message& msg)
{
  if (msg.length < 2) {
    return;
  }

  uint8_t position = msg.payload[0];
  uint8_t offset = msg.payload[1];

  switch (position) {
    case 0x00:
      gimbal.move_left(offset);
      break;
    case 0x01:
      gimbal.move_right(offset);
      break;
    case 0x02:
      gimbal.move_up(offset);
      break;
    case 0x03:
      gimbal.move_down(offset);
      break;
  }
}

void handle_heartbeat_can_command(hal::can_message&){
  // TODO: implement real heartbeat parsing later
}

void application()
{
  // using namespace hal::literals;
  // using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto watchdog = resources::watchdog();
  auto i2c = resources::i2c();
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();

  // constexpr auto wait_time = 5s;
  // if (watchdog->check_flag()) {
  //   hal::print(*console, "Reset by watchdog\n");
  //   watchdog->clear_flag();
  // } else {
  //   hal::print(*console, "Non-watchdog reset\n");
  // }

  // watchdog->set_countdown_time(wait_time);
  // watchdog->start();

  // Initialize ICM
  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c);
  icm_device->init_mag();
  icm_device->auto_offsets();

  // hal::actuator::rc_servo::settings const gimbal_servo_settings{
  //   .frequency = 50,
  //   .min_angle = 0,
  //   .max_angle = 180,
  //   .min_microseconds = MIN_PULSEWIDTH_RANGE,
  //   .max_microseconds = MAX_PULSEWIDTH_RANGE,
  // };

  // Initialize servos
  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_0,
    gimbal_servo_settings);

  auto p_y_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_1,
    gimbal_servo_settings);

  // auto y_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
  //   resources::driver_allocator(),
  //   *mast_servo_pwm_channel_1,
  //   gimbal_servo_settings);

  // Initialize Gimbal
  gimbal mast(p_x_servo,
              p_y_servo,
              icm_device,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);

  // static constexpr auto baudrate = 100.0_kHz;

  can_bus_manager->baud_rate(baudrate);
  // constexpr auto allowed_id = 0x300;

  // Allowing these specific IDs to read
  can_id_filter->allow(gimbal_read_id);
  can_id_filter->allow(heartbeat_read_id);
  hal::print<64>(
    *console,
    "Allowing ID to read [0x%lX] and [0x%lX] through the filter!\n",
    gimbal_read_id,
    heartbeat_read_id);

  hal::can_message_finder gimbal_msg_finder(*can_transceiver, gimbal_read_id);
  hal::can_message_finder heartbeat_msg_finder(*can_transceiver,
                                               heartbeat_read_id);

  // constexpr auto wait_time = 5s;
  if (watchdog->check_flag()) {
    hal::print(*console, "Reset by watchdog\n");
    watchdog->clear_flag();
  } else {
    hal::print(*console, "Non-watchdog reset\n");
  }

  watchdog->set_countdown_time(wait_time);
  watchdog->start();

  auto last_time = clock->uptime();

  while (true) {
    // Get current current time
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
    for (auto m = gimbal_msg_finder.find(); m.has_value();
         m = gimbal_msg_finder.find()) {
            handle_gimbal_can_command(mast, *m);

    }
    for (auto m = heartbeat_msg_finder.find(); m.has_value();
         m = heartbeat_msg_finder.find()) {
          handle_heartbeat_can_command(*m);
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

    std::array<hal::byte, 8> accel_can_payload = {
      static_cast<uint8_t>(raw_accel_int16.x & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_accel_int16.x) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_accel_int16.y & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_accel_int16.y) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_accel_int16.z & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_accel_int16.z) >> 8) &
                           0xFF)
    };

    std::array<hal::byte, 8> gyro_can_payload = {
      static_cast<uint8_t>(raw_gyro_int16.x & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_gyro_int16.x) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_gyro_int16.y & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_gyro_int16.y) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_gyro_int16.z & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_gyro_int16.z) >> 8) &
                           0xFF)
    };

    std::array<hal::byte, 8> mag_can_payload = {
      static_cast<uint8_t>(raw_mag_int16.x & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_mag_int16.x) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_mag_int16.y & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_mag_int16.y) >> 8) &
                           0xFF),
      static_cast<uint8_t>(raw_mag_int16.z & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(raw_mag_int16.z) >> 8) & 0xFF)
    };

    std::array<hal::byte, 8> heartbeat_can_payload = { imu_status,
                                                        lcd_status };

    can_transceiver->send(hal::can_message{ .id = imu_accel_reply_id,
                                            .extended = false,
                                            .remote_request = false,
                                            .length = 6,
                                            .reserved0 = 0,
                                            .payload = accel_can_payload });

    can_transceiver->send(hal::can_message{ .id = imu_gyro_reply_id,
                                            .extended = false,
                                            .remote_request = false,
                                            .length = 6,
                                            .reserved0 = 0,
                                            .payload = gyro_can_payload });

    can_transceiver->send(hal::can_message{ .id = imu_mag_reply_id,
                                            .extended = false,
                                            .remote_request = false,
                                            .length = 6,
                                            .reserved0 = 0,
                                            .payload = mag_can_payload });

    can_transceiver->send(
      hal::can_message{ .id = heartbeat_reply_id,
                        .extended = false,
                        .remote_request = false,
                        .length = 2,
                        .reserved0 = 0,
                        .payload = heartbeat_can_payload });

    // Update the pitch servo
    mast.update_y_servo(dt);

    // Reset the Watchdog countdown timer
    watchdog->reset();

    hal::delay(*clock, 2ms);
  }
}
}  // namespace sjsu::hub