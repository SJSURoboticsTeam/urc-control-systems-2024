// copied from drivers/applications/h_bridge_demo.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <type_traits>

#include "../hardware_map.hpp"

#include "../include/can_messaging.hpp"
#include "../include/bldc_servo.hpp"


using namespace std::chrono_literals;
namespace sjsu::perseus {
/*
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
  set_position = 0x12,
  set_velocity = 0x13,
  stop = 0x22,  // hard stop the servo to be 0
                // readers
  read_position = 0x20,
  read_velocity = 0x21,
  // setters
  clamp_speed = 0x30,
  set_pid_position = 0x31,
  set_pid_velocity = 0x32
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
                         [[maybe_unused]] hal::v5::strong_ptr<hal::can_message> response)
{
  switch (static_cast<action>(p_message.payload[0])) {
    // case action::read_position: {
    //   auto current_position = bldc->get_current_position();
    //   response->length = 3;
    //   response->payload[0] =
    //     static_cast<hal::byte>(action::read_position) + 0x50;
    //   response->payload[1] = (current_position >> 8) & 0xff;
    //   response->payload[1] = current_position & 0xff;
    //   break;
    // }
    case action::set_position: {
      auto target_position = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_target_position(target_position);
      break;
    }
    case action::clamp_speed: {
      auto target_speed = p_message.payload[1] << 8 | p_message.payload[2];
      bldc->set_clamped_speed(target_speed);
      break;
    }
    case action::read_velocity: {
      // auto current_velocity = bldc->get_current_velocity_percentage();
      // response->payload[0] =
      //   static_cast<hal::byte>(action::read_velocity) + 0x50;
      // response->payload[1] =
      //   (current_velocity >> 8) & 0xFF;  // HIGH BYTE FIRST // HIGH BYTE FIRST
      // response->payload[2] = current_velocity & 0xFF;  // LOW BYTE SECOND
      break;
    }
    case action::stop:
      bldc->set_current_velocity(0);
      break;
    case action::set_pid_position: {
      bldc_perseus::PID_settings settings = {
        .kp =
          static_cast<float>(p_message.payload[1] << 8 | p_message.payload[2]),
        .ki =
          static_cast<float>(p_message.payload[3] << 8 | p_message.payload[4]),
        .kd =
          static_cast<float>(p_message.payload[5] << 8 | p_message.payload[6])
      };
      bldc->update_pid_position(settings);
      break;
    }
    case action::set_pid_velocity: {
      bldc_perseus::PID_settings settings = {
        .kp =
          static_cast<float>(p_message.payload[1] << 8 | p_message.payload[2]),
        .ki =
          static_cast<float>(p_message.payload[3] << 8 | p_message.payload[4]),
        .kd =
          static_cast<float>(p_message.payload[5] << 8 | p_message.payload[6])
      };
      bldc->update_pid_velocity(settings);
      break;
    }
    default:
      hal::operation_not_supported(nullptr);
  }
}
*/
// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  auto can_transceiver = resources::can_transceiver();
  auto bus_manager = resources::can_bus_manager();
  bus_manager->baud_rate(1.0_MHz);
  auto can_id_filter = resources::can_identifier_filter();
  hal::u16 servo_address = can_perseus::servo_address::track_servo;
  hal::can_message_finder can_finder(*can_transceiver, 0x110);

  can_id_filter->allow(0x110);
  hal::print(*console, "CAN message finder initialized...\n");
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");
  can_perseus servo_can(servo_address); 
  hal::print(*console, "Servo can creature setup...\n");

  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  hal::print(*console, "BLDC Servo initialized...\n");

  while (true) {

    // print velocity 
    hal::u64 curr_time = clock->uptime(); 
    float curr_angle = servo.get_current_position();
    hal::u64 dt = curr_time - servo.m_last_clock_check; 
    float da = curr_angle - servo.m_prev_encoder_value; 
    float da_dt = 10 * da / static_cast<float>(dt); 
    hal::print<128>(*console, "VELOCITY: %x", da_dt);
      

    auto optional_message = can_finder.find();
    if (optional_message) {
      hal::print<128>(*console, "%x%X Servo received a message", servo_address);
      servo_can.print_can_message(*console, *optional_message);
      auto response = hal::v5::make_strong_ptr<hal::can_message>(resources::driver_allocator(), hal::can_message{});
      servo_can.can_perseus::process_can_message(*optional_message, servo_ptr, response);
      can_finder.transceiver().send(*response);
    }
    servo.m_last_clock_check = curr_time; 
    servo.m_prev_encoder_value = curr_angle; 
  } 
}
}  // namespace sjsu::perseus