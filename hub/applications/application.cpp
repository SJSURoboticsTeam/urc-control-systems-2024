#include <libhal-actuator/rc_servo.hpp>
#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>


#include <resource_list.hpp>
#include "../include/gimbal.hpp"

// Pulse width range for FT5325M
#define MIN_PULSEWIDTH_RANGE 900
#define MAX_PULSEWIDTH_RANGE 2100

// static hal::time_duration const wd_countdown_timer = <recovery - expr>(5);
namespace sjsu::hub {

void handle_can_command(gimbal& gimbal, hal::can_message& msg)
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
void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto watchdog = resources::watchdog();
  auto i2c = resources::i2c();
  auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();

  constexpr auto wait_time = 5s;
  if (watchdog->check_flag()) {
    hal::print(*console, "Reset by watchdog\n");
    watchdog->clear_flag();
  } else {
    hal::print(*console, "Non-watchdog reset\n");
  }

  watchdog->set_countdown_time(wait_time);
  watchdog->start();

  auto icm_device = hal::v5::make_strong_ptr<hal::sensor::icm20948>(
    resources::driver_allocator(), *i2c);
  icm_device->init_mag();
  icm_device->auto_offsets();

  hal::actuator::rc_servo::settings const gimbal_servo_settings{
    .frequency = 50,
    .min_angle = 0,
    .max_angle = 180,
    .min_microseconds = MIN_PULSEWIDTH_RANGE,
    .max_microseconds = MAX_PULSEWIDTH_RANGE,
  };
  auto p_x_servo = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_0,
    gimbal_servo_settings);

  auto y_servo_ptr = hal::v5::make_strong_ptr<hal::actuator::rc_servo>(
    resources::driver_allocator(),
    *mast_servo_pwm_channel_1,
    gimbal_servo_settings);

  gimbal mast(p_x_servo,
              p_x_servo,
              icm_device,
              gimbal_servo_settings.min_angle,
              gimbal_servo_settings.max_angle);

  static constexpr auto baudrate = 100.0_kHz;

  can_bus_manager->baud_rate(baudrate);
  constexpr auto allowed_id = 0x300;
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*can_transceiver, 0x300);

  auto last_time = clock->uptime();
  while (true) {
    auto now_time = clock->uptime();
    float dt = std::chrono::duration<float>(now_time - last_time).count();
    if (dt <= 0.0f) dt = 1e-6f;
    last_time = now_time;
    for (auto m = message_finder.find(); m.has_value();
         m = message_finder.find()) {
    }
    mast.update_y_servo(dt);
    
    watchdog->reset();

    hal::delay(*clock, 2ms);
  }
}
}  // namespace sjsu::hub