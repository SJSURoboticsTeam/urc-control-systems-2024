#include "can_messaging.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>

#include <can_util.hpp>
#include <bldc_servo_settings.hpp>
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
  bldc_perseus::physical_servo_values physical_servo_values;
  // variables used to initalize can_perseus
  can_perseus::servo_address allowed_id; 
  hal::u16 listen_prev = 0x00; 
  auto switches_ptr = resources::switches(); 
  allowed_id = static_cast<can_perseus::servo_address>(static_cast<hal::u16>(switches_ptr->read_switch_value()) + 0x120); 
  // set servo values according to switch 
  switch (allowed_id) {
    case can_perseus::track_servo:
      pid_settings = track_pid_settings; 
      physical_servo_values = track_physical_servo_values; 
      break; 
    case can_perseus::shoulder_servo:
      pid_settings = shoulder_pid_settings; 
      physical_servo_values = shoulder_physical_servo_values; 
      break; 
    case can_perseus::elbow_servo:
      pid_settings = elbow_pid_settings;
      physical_servo_values = elbow_physical_servo_values; 
      listen_prev = can_perseus::shoulder_servo; 
      break; 
    case can_perseus::wrist_left:
      pid_settings = wrist_left_pid_settings;
      physical_servo_values = wrist_left_physical_servo_values; 
      listen_prev = can_perseus::elbow_servo; 
      break;
    case can_perseus::wrist_right:
      pid_settings = wrist_right_pid_settings;
      physical_servo_values = wrist_right_physical_servo_values; 
      listen_prev = can_perseus::elbow_servo;
      break; 
    case can_perseus::end_effector:
      pid_settings = end_effector_pid_settings;
      physical_servo_values = end_effector_physical_servo_values; 
      break; 
    default: 
      hal::print(*console, "Address does not exist. Exiting.\n");
      return; 
  }

  // create bldc_perseus 
  auto servo_ptr = resources::servo(); 
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_physical_servo_values(physical_servo_values); 
  servo_ptr->get_actual_position();
  hal::print(*console, "pre-can\n"); 
  
  // create can_perseus 
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_id_filter = resources::can_identifier_filter();
  hal::v5::strong_ptr<hal::can_mask_filter> can_mask_filter = resources::can_mask_filter(); 
  can_perseus servo_can(allowed_id, listen_prev, 1_MHz, can_transceiver, can_bus_manager,  can_id_filter, can_mask_filter); 
    
  hal::print(*console, "Begin.\n");
  
  // start loop
  bool new_action = false; 

  while (true) {

    // receive message 
    std::optional<hal::can_message> msg = servo_can.check_for_mc_message(); 
  
    // react to message 
    if (msg) {
      drivers::can_util::print_can_message(*console, *msg);
      servo_can.process_can_message(*msg, *servo_ptr); 
      hal::print<64>(*console, "Action: %x \n", servo_ptr->get_active_action());
      new_action = true; 
    }

    servo_ptr->periodic_action(new_action); 
    new_action = false; 


  }
  
}
}  // namespace sjsu::perseus