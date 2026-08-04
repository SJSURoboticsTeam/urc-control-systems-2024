// derived from drivers/applications/velocity_test.cpp

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
#include <switches.hpp>
// #include "can_perseus_applications.cpp"
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {


void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // general 
  auto clock = resources::clock();
  auto console = resources::console();

  // variables used to initalize bldc_perseus
  bldc_perseus::PID_settings pid_settings;
  bldc_perseus::servo_values servo_values;
  // variables used to initalize can_perseus
  can_perseus::servo_address allowed_id; 
  hal::u8 listen_prev = 0; 
  // figure out who this is by the value of the switches 
  auto switches = switches_bldc(resources::switch_g1(), 
                    resources::switch_g2(), 
                    resources::switch_g3(), 
                    resources::switch_g4(), 
                    resources::switch_g5(), 
                    resources::switch_g6()); 
  allowed_id = static_cast<can_perseus::servo_address>(static_cast<hal::u16>(switches.read_switch_value()) + 0x120); 
  // set servo values according to switch 
  switch (allowed_id) {
    case can_perseus::track_servo:
      // pid
      pid_settings = {
        .kp = 0.04, 
        .ki = 0.00, 
        .kd = 0.00,
      };
      // servo 
      servo_values = {
        .gear_ratio = 16915.5, // 751.8 * 1 / 2 * 360 / 8 (for mm) 
        .angle_offset = 0, 
        .fight_gravity = 0, 
        .high_clamped_value = 0.3, 
        .low_clamped_value = -0.3, 
        .flipped_direction = false 
      }; 
      // listening to previous joint?  
      listen_prev = 0; 
      break; 
    case can_perseus::shoulder_servo:
      // pid
      pid_settings = {
        .kp = 0.5,
        .ki = 0.00,
        .kd = 0.007,
      };
      // servo 
      servo_values = {
        .gear_ratio = 73935.4, // 5281.1 * 28 / 2
        .angle_offset = 0, 
        .fight_gravity = 0, 
        .high_clamped_value = 0.3, 
        .low_clamped_value = -0.3, 
        .flipped_direction = false 
      }; 
      // listening to previous joint?  
      listen_prev = 0; 
      break; 
    case can_perseus::elbow_servo:
      // pid
      pid_settings = {
        .kp = 0.01, 
        .ki = 0.00, 
        .kd = 0.005,
      };
      // servo 
      servo_values = {
        .gear_ratio = 5281.1, // 5281.1 * 2 / 2
        .angle_offset = 0, 
        .fight_gravity = 0.15, 
        .high_clamped_value = 0.1, 
        .low_clamped_value = -0.3, 
        .flipped_direction = false 
      }; 
      // listening to previous joint?  
      listen_prev = 1; 
      break; 
    case can_perseus::wrist_left:
      // pid
      pid_settings = {
        .kp = 0.005,
        .ki = 0.00,
        .kd = 0.00,
      };
      // servo 
      servo_values = {
        .gear_ratio = 2640.55, // 5281.1 * 1 / 2
        .angle_offset = 0, 
        .fight_gravity = 0.2, 
        .high_clamped_value = 0.3, 
        .low_clamped_value = -0.3, 
        .flipped_direction = false 
      }; 
      // listening to previous joint?  
      listen_prev = 1; 
      break;
    case can_perseus::wrist_right:
      // pid
      pid_settings = {
        .kp = 0.005,
        .ki = 0.00,
        .kd = 0.00,
      };
      // servo 
      servo_values = {
        .gear_ratio = 2640.55, // 5281.1 * 1 / 2
        .angle_offset = 0, 
        .fight_gravity = 0.2, 
        .high_clamped_value = 0.3, 
        .low_clamped_value = -0.3, 
        .flipped_direction = true 
      }; 
      // listening to previous joint? 
      listen_prev = 2;
      break; 
    case can_perseus::end_effector:
      // pid
      pid_settings = {
        .kp = 0.05, 
        .ki = 0.00, 
        .kd = 0.1,
      };
      // servo 
      servo_values = {
        .gear_ratio = 73935.4, // 5281.1 * 28 / 2
        .angle_offset = 0, 
        .fight_gravity = 0, 
        .high_clamped_value = 0.3, 
        .low_clamped_value = -0.3, 
        .flipped_direction = false 
      }; 
      // listening to previous joint?  
      listen_prev = 0; 
      break; 
    default: 
      hal::print(*console, "Address does not exist. Exiting.\n");
      return; 
  }

  // create bldc_perseus 
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  bldc_perseus servo(h_bridge, encoder);
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_servo_values(servo_values); 
  servo_ptr->set_actual_position();
  
  // create can_perseus 
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();
  can_perseus servo_can(allowed_id, 1_MHz, listen_prev, can_transceiver, can_bus_manager,  can_id_filter); 
  auto can_ptr = hal::v5::make_strong_ptr<decltype(servo_can)>(resources::driver_allocator(), std::move(servo_can));
  
  hal::print(*console, "Begin.\n");
  
  // start the forever loop 
  // can_application(servo_ptr, can_ptr); 
  bool new_action = false; 

  while (true) {

    // receive message 
    std::optional<hal::can_message> msg = can_ptr->check_for_mc_message(); 
  
    // react to message 
    if (msg) {
      can_ptr->print_can_message(*console, *msg);
      can_ptr->process_can_message(*msg, servo_ptr); 
      hal::print<64>(*console, "Action: %x \n", servo_ptr->get_active_action());
      new_action = true; 
    }

    // can_ptr->periodic_action(servo_ptr->get_active_action(), servo_ptr); 
    servo_ptr->periodic_action(new_action); 
    new_action = false; 
    hal::delay(*clock, 50ms); 


  }
  
}
}  // namespace sjsu::perseus