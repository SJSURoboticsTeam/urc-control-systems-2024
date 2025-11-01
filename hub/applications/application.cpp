#include "../hardware_map.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

// Pulse width range for FT5325M
#define MIN_PULSEWIDTH_RANGE 900
#define MAX_PULSEWIDTH_RANGE 2100
static hal::time_duration const wd_countdown_timer =
  std::chrono::nanoseconds(5000000000);  // 5 secs

// static hal::time_duration const wd_countdown_timer = <recovery - expr>(5);
namespace sjsu::hub {
hal::actuator::rc_servo::settings const gimbal_servo_settings{
  .frequency = 50,
  .min_angle = 0,
  .max_angle = 180,
  .min_microseconds = MIN_PULSEWIDTH_RANGE,
  .max_microseconds = MAX_PULSEWIDTH_RANGE,
};
void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "Received new hal::can_message { \n"
                 "    id: 0x%lX,\n"
                 "    length: %u \n"
                 "    payload = [ ",
                 p_message.id,
                 p_message.length);

  for (auto const& byte : p_message.payload) {
    hal::print<8>(p_console, "0x%02X, ", byte);
  }

  hal::print(p_console, "]\n}\n");
}
void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  auto console = resources::console();

  //   auto mast_servo_pwm_channel_0 = resources::mast_servo_pwm_channel_0();
  //   auto mast_servo_pwm_channel_1 = resources::mast_servo_pwm_channel_1();
  //   auto i2c = resources::i2c();

  // Initializing ICM
  hal::sensor::icm20948 icm(*i2c);

  hal::print(*console, "test1\n");

  // List of watchdog commands to use
  hal::stm32f1::independent_watchdog hub_watchdog{};
  hub_watchdog.set_countdown_time(wd_countdown_timer);
  // hub_watchdog.start(); // Start watchdog countdown
  // hub_watchdog.reset();  // Reset watchdog countdown
  // hub_watchdog.check_flag(); // True if it reaches countdown; False if its
  // still during the countdown
  // hub_watchdog.clear_flag();  // Clears the flag

  // Initializing the servos for the gimbal (takes in pwm not pwm16_channel)
  // Ensure the connections to servo are correct
  //   hal::actuator::rc_servo p_x_servo(*mast_servo_pwm_channel_0,
  //                                     gimbal_servo_settings);
  //   hal::actuator::rc_servo p_y_servo(*mast_servo_pwm_channel_1,
  //                                     gimbal_servo_settings);

  // Initializing Watchdog

  //   hal::print(*console, "Starting Application!\n");
  //   hal::print(*console, "Will reset after ~10 seconds\n");

  //   for (int i = 0; i < 10; i++) {
  //     // Print message
  //     hal::print(*console, "Hello, World\n");
  //   }

  //   hal::print(*console, "Resetting!\n");
  //   hal::delay(*clock, 100ms);
  //   resources::reset();

  static constexpr auto baudrate = 100.0_kHz;

  can_bus_manager->baud_rate(baudrate);
  can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });
  constexpr auto allowed_id = 0x300;
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*can_transceiver, 0x300);

  while (true) {

    for (auto m = message_finder.find(); m.has_value();
         m = message_finder.find()) {
      print_can_message(*console, *m);
      auto& msg = *m;
      // if(m.hasvalue) reset watchdog timer
      if (msg.length == 2) {
        uint8_t position = msg.payload[0];
        // uint8_t offset = msg.payload[1];
        switch (position) {
          case 0x00:
            // move left
            break;
          case 0x01:
            // move right
            break;
          case 0x02:
            // move up
            break;
          case 0x03:
            // move down
            break;
        }
      }
    }
  }
}
}  // namespace sjsu::hub