#include "homing.hpp"
#include <libhal/can.hpp>
#include <libhal/units.hpp>

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

float get_angle(hal::u32 p_id, hal::can_transceiver& p_can)
{

  send_custom_message(p_id, p_can, 8, { 0x60, 0x0, 0, 0, 0, 0, 0, 0 });
  hal::can_message_finder angle_reader(p_can, 0x248);
  std::optional<hal::can_message> angle = angle_reader.find();
  while (!angle) {
    angle = angle_reader.find();
  }
  float finalAngle = angle->payload[4] | angle->payload[5] >> 8 |
                     angle->payload[6] >> 16 | angle->payload[7] >> 24;
  return finalAngle;
}

void home(std::span<steering_module> legs,
          std::span<start_wheel_setting> setting_span,
          hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal)
{
  for (size_t i = 0; i < legs.size(); i++) {
    auto motor = legs[i].steer->acquire_motor(5.0_rpm);
    auto servo = legs[i].steer->acquire_servo(5.0_rpm);
    // auto rotation_sensor = legs[i].steer->acquire_rotation_sensor();
    // hal::print<2048>(terminal, "start angle: %f\n", rotation_sensor.read());

    hal::u32 id = setting_span[i].steer_id;
    auto print_feedback = [&terminal, id, &can]() {
      hal::print<2048>(terminal,
                       "[%u] =================================\n"
                       "shaft angle = %f deg\n"
                       "\n\n",
                       get_angle(id, can));
    };

    // motor.power(0.01);
    // motor.power(-0.01);
    // hal::print(terminal, "Servo moving\n");
    // hal::delay(clock, 10ms);
    // hal::degrees current_pos = rotation_sensor.read().angle;
    // print_feedback();

    // while (!legs[i].limit_switch.value()->level()) {
    //   if (setting_span[i].reversed) {
    //     servo.position(current_pos++);
    //     hal::delay(clock, 10ms);
    //   } else {
    //     servo.position(current_pos--);
    //     hal::delay(clock, 10ms);
    //   }
    // }
    // motor.power(0);
    // print_feedback();

    print_feedback();
    float start_angle = get_angle(setting_span[i].steer_id, can);
    while (!legs[i].limit_switch.value()->level()) {
      if (setting_span[i].reversed) {
        motor.power(0.3);
        hal::delay(clock, 10ms);
      } else {
        motor.power(-0.3);
        hal::delay(clock, 10ms);
      }
    }
    print_feedback();
    float stop_angle = get_angle(setting_span[i].steer_id, can);

    motor.power(0.0);
    hal::delay(clock, 1000ms);

    hal::print(terminal, "Stopped\n");

    int deg = 0;
    while (fabs(start_angle - stop_angle) > deg) {
      servo.position(stop_angle);
      stop_angle = setting_span[i].reversed ? stop_angle + 1 : stop_angle - 1;
      deg++;
      hal::print(terminal, "Stopped\n");
    }
    motor.power(0.0);
    hal::delay(clock, 1000ms);
    // hal::delay(clock, 500ms);
    // auto position = rotation_sensor.read();
    // hal::print<1024>(terminal, "Current position feedback: %f\n", position);

    // hal::delay(clock, 500ms);

    // servo.position(position.angle + setting_span[i].offset);
    // position = rotation_sensor.read();
    // hal::print<1024>(
    //   terminal, "After offset position feedback: %f\n", position);

    // can message
    // send_custom_message(
    //   setting_span[i].steer_id, can, 8, { encoder_zero_command });
    // hal::delay(clock, 500ms);
    // send_custom_message(setting_span[i].steer_id, can, 8, { reset_command });
    // hal::delay(clock, 500ms);
    // send_custom_message(setting_span[i].steer_id, can, 8, { reset_command });
  }
}
}  // namespace sjsu::drive