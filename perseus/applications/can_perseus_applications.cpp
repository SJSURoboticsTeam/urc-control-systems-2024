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

void can_application(hal::v5::strong_ptr<bldc_perseus> const& servo_ptr, 
                      hal::v5::strong_ptr<can_perseus> const& can_ptr )
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // general 
  auto clock = resources::clock();
  auto console = resources::console();
  
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