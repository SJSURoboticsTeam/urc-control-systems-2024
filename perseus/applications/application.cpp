// copied from drivers/applications/h_bridge_demo.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include "../../drivers/include/h_bridge.hpp"
#include "../hardware_map.hpp"
#include <bldc_servo.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
using namespace std::chrono_literals;
namespace sjsu::perseus {

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

enum class action : hal::byte
{
  // actuators
  actuate_torque = 0x10,  // (?)
  actuate_position = 0x12,
  stop = 0x22,  // hard stop the servo to be 0
                // readers
  read_position = 0x20,
  read_velocity = 0x21,
  // setters
  clamp_speed = 0x30,
  set_pid = 0x31
};
enum servo_address : hal::u16
{
  track_servo = 0x120,
  shoulder_servo = 0x121,
  elbow_servo = 0x122,
  wrist_pitch = 0x123,
  wrist_roll = 0x124,
  clamp = 0x125
};

void process_can_message(hal::can_message const& p_message,
                         hal::v5::strong_ptr<bldc_perseus> bldc,
                         hal::v5::optional_ptr<hal::can_message> response)
{
  switch (static_cast<action>(p_message.payload[0])) {
    case action::read_position: {
      auto current_position = bldc->get_current_position();
      response->payload[0] =
        static_cast<hal::byte>(action::read_position) + 0x50;
      response->payload[1] = (current_position >> 8) & 0xff;
      response->payload[1] = current_position & 0xff;
      break;
    }
    case action::actuate_position: {
      auto target_position = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_target_position(target_position);
      break;
    }
    case action::clamp_speed: {
      auto target_speed = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_clamped_speed(target_speed);
      break;
    }
    case action::actuate_torque: {
      // i think what i mean by this is that we are updating the torque (force
      // on this joint)
      // so we need to calculate equal opposite velocity that we need the motor
      // to run to not droop
      break;
    }
    case action::read_velocity: {
      auto current_velocity = bldc->get_current_velocity();
      response->payload[0] =
        static_cast<hal::byte>(action::read_velocity) + 0x50;
      response->payload[1] =
        (current_velocity >> 8) & 0xFF;  // HIGH BYTE FIRST // HIGH BYTE FIRST
      response->payload[2] = current_velocity & 0xFF;  // LOW BYTE SECOND
      break;
    }
    case action::stop:
      bldc->set_current_velocity(0);
      break;
    case action::set_pid: {
      bldc_perseus::PID_settings settings = {
        .kp =
          static_cast<float>(p_message.payload[1] << 8 | p_message.payload[2]),
        .ki =
          static_cast<float>(p_message.payload[3] << 8 | p_message.payload[4]),
        .kd =
          static_cast<float>(p_message.payload[5] << 8 | p_message.payload[6])
      };
      bldc->update_pid_settings(settings);
      break;
    }
    default:
      hal::operation_not_supported(nullptr);
  }
}

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto a_low = resources::output_pin_0();
  auto b_low = resources::output_pin_1();
  auto a_high = resources::pwm_channel_0();
  auto b_high = resources::pwm_channel_1();
  auto h_bridge = sjsu::drivers::h_bridge({ a_high, a_low }, { b_high, b_low });
  auto h_bridge_ptr = hal::v5::make_strong_ptr<decltype(h_bridge)>(
    resources::driver_allocator(), std::move(h_bridge));
  auto encoder = resources::encoder();
  auto can_transceiver = resources::can_transceiver();
  auto bus_manager = resources::can_bus_manager();
  auto console = resources::console();
  hal::u16 servo_address =
    servo_address::track_servo;  // try to input this somehow idk but this needs
                                 // to change depending on device flashing
  auto can_finder = resources::can_finder(can_transceiver, servo_address);
  bus_manager->baud_rate(1.0_MHz);
  auto servo = hal::v5::make_strong_ptr<bldc_perseus>(
    resources::driver_allocator(), h_bridge_ptr, encoder);

  while (true) {
    // forever can loop
    // basically check if we have any message
    // if we have then update target struct
    // in each loop check current position and update velocity using PID and
    // update the current position

    auto optional_message = can_finder->find();
    if (optional_message) {
      hal::print<128>(*console, "%x%X Servo received a message", servo_address);
      print_can_message(*console, *optional_message);
      hal::v5::optional_ptr<hal::can_message> response;
      response->length = 8;
      process_can_message(*optional_message, servo, response);
      if (response) {
        can_finder->transceiver().send(*response);
      }
    }
    // this is a very preliminary version
    servo->set_target_velocity(
      (servo->get_target_position() - servo->get_current_position()));
  }
}
}  // namespace sjsu::perseus