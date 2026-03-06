// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {

enum servo_address : hal::u16
{
track_servo = 0x121,
  shoulder_servo = 0x122,
  elbow_servo = 0x123,
  wrist_left = 0x124,
  wrist_right = 0x125,
    clamp = 0x126,
};

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "📩 Received new hal::can_message { \n"
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

// bool action_loop(hal::v5::strong_ptr<bldc_perseus> bldc,
//                         hal::v5::strong_ptr<can_perseus> can,
//                         hal::v5::strong_ptr<hal::can_message> response,
//                         uint32_t action, 
//                         bool new_action, 
//                         int delay_counter)
// {   
//   bool send = false; 
//   auto console = resources::console();
//   switch (static_cast<can_perseus::action>(action)) {
//     case can_perseus::action::homing: {
//       bldc->home_encoder(); 
//       if (delay_counter >= 10) {
//         delay_counter = 0; 
//         hal::u16 t = can->can_perseus::floating_to_fixed_point(bldc->get_reading_position(), 6); 
//         response->length = 8;
//         response->payload[0] = 0x20 + 0x50; 
//         response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
//         response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
//         send = true; 
//       }
//       break; 
//     }
//     case can_perseus::action::set_position: {
//       if (new_action) {
//         bldc->update_position(1); 
//       }
//       else {
//         bldc->update_position(0); 
//       }
//       if (delay_counter >= 10) {
//         delay_counter = 0; 
//         hal::u16 t = can->can_perseus::floating_to_fixed_point(bldc->get_reading_position(), 6); 
//         response->length = 8;
//         response->payload[0] = 0x20 + 0x50; 
//         response->payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
//         response->payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
//         send = true; 
//       }
//       break;
//     }
//     default:
//       break; 
//   }
//   return send; 
// }



// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2
void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // gen
  auto clock = resources::clock();
  auto console = resources::console();
  // can
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  // CHANGE SERVO
  // constexpr servo_address allowed_id = track_servo; 
  // constexpr servo_address allowed_id = elbow_servo; 
  constexpr servo_address allowed_id = shoulder_servo; 
  // constexpr servo_address allowed_id = wrist_left; 
  // constexpr servo_address allowed_id = wrist_right;
  static constexpr auto baudrate = 1_MHz;
  can_perseus servo_can(allowed_id, baudrate, can_transceiver, can_bus_manager, can_interrupt, can_id_filter); 
  auto can_ptr = hal::v5::make_strong_ptr<decltype(servo_can)>(resources::driver_allocator(), std::move(servo_can));
  hal::print(*console, "Servo can creature setup...\n");
  // bldc
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  // CHANGE SERVO
  servo_ptr->set_pid_clamped_power(0.3); 
  // servo_ptr->set_pid_clamped_power(0.5); 

  hal::print(*console, "CAN IT\n");


  // CHANGE SERVO
  // // track
  // bldc_perseus::PID_settings pid_settings = {
  //   .kp = 0.04, 
  //   .ki = 0.00, 
  //   .kd = 0.00,
  // };
  // // elbow
  // bldc_perseus::PID_settings pid_settings = {
  //   .kp = 0.2, 
  //   .ki = 0.00, 
  //   .kd = 0.002,
  // };
  // shoulder
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.5,
    .ki = 0.00,
    .kd = 0.005,
  };
  // // wrist
  //   bldc_perseus::PID_settings pid_settings = {
  //   .kp = 0.05,
  //   .ki = 0.00,
  //   .kd = 0.008,
  // };

  servo_ptr->update_pid_position(pid_settings);

  servo_ptr->set_actual_position();
  
  
  bool new_action = false; 
  int delay_counter = 0; 

  while (true) {

    // receive message 
    std::optional<hal::can_message> msg = can_ptr->check_for_mc_message(); 
  
    // react to message 
    if (msg) {
      print_can_message(*console, *msg);
      can_ptr->process_can_message(*msg, servo_ptr); 
      hal::print<64>(*console, "Action: %x \n", servo_ptr->get_reading_action());
      new_action = true; 
    }

    // continue action 
    if(servo_ptr->get_reading_action() != 0) {
      if (delay_counter >= 6) {
        delay_counter = 0;
        // can_ptr->repeating_action_can(servo_ptr->get_reading_action(), servo_ptr); 
      }
      servo_ptr->repeating_action_bldc(new_action); 
    }
    
    new_action = false; 
    delay_counter++; 
    hal::delay(*clock, 50ms); 


  }
  
}
}  // namespace sjsu::perseus