#include "homing.hpp"

using namespace std::chrono_literals;
using namespace hal::literals;
namespace sjsu::drive {
void send_custom_message(hal::u32 p_id,
                         hal::can_transceiver& p_can,
                         hal::u8 p_length,
                         std::array<hal::byte, 8>&& p_payload)
{
  const hal::can_message message = { .id = p_id,
                                     .length = p_length,
                                     .payload = p_payload };
  p_can.send(message);
}


void home(std::span<steering_module> legs,
          std::span<start_wheel_setting> setting_span,
          hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal)
{
  for (size_t i = 0; i < legs.size(); i++) {
    
    auto current_sensor = legs[i].steer->acquire_current_sensor();
    auto motor = legs[i].steer->acquire_motor(5.0_rpm);
    auto servo = legs[i].steer->acquire_servo(5.0_rpm);
    auto rotation_sensor = legs[i].steer->acquire_rotation_sensor();
    auto curr = current_sensor.read();
    auto print_feedback =
      [&terminal, &rotation_sensor, &current_sensor]() {
        hal::print<2048>(terminal,
                         "[%u] =================================\n"
                         "shaft angle = %f deg\n"
                         "current = %f Amps\n"
                         "\n\n",
                         rotation_sensor.read().angle,
                         current_sensor.read());
      };
    while (std::abs(curr) < 12) {
      if (setting_span[i].reversed) {
        motor.power(0.3);
        hal::delay(clock, 10ms);
      } else {
        hal::print(terminal, "here");
        motor.power(-0.3);
        hal::delay(clock, 10ms);
      }
      // hal::print<1028>(terminal, "current = %f Amps\n", curr);
      print_feedback();
      curr = current_sensor.read();
    }
    // can message
    hal::delay(clock, 500ms);

    auto position = rotation_sensor.read();

    hal::print<1024>(terminal, "Current position feedback: %f\n", position);
    
    hal::delay(clock, 500ms);

    servo.position(position.angle + setting_span[i].offset);
    position = rotation_sensor.read();
    hal::print<1024>(terminal, "After offset position feedback: %f\n", position);

    
    send_custom_message(
      setting_span[i].steer_id, can, 8, { encoder_zero_command });
    hal::delay(clock, 500ms);
    send_custom_message(setting_span[i].steer_id, can, 8, { reset_command });
    hal::delay(clock, 500ms);
    send_custom_message(0x205, can, 8, { reset_command });
  }
}
}  // namespace sjsu::drive