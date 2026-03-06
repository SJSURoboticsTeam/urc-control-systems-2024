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

void can_application(hal::v5::strong_ptr<bldc_perseus> servo_ptr, hal::v5::strong_ptr<can_perseus> can_ptr)
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // gen
  auto clock = resources::clock();
  auto console = resources::console();
  
  bool new_action = false; 
  int delay_counter = 0; 

  while (true) {

    // receive message 
    std::optional<hal::can_message> msg = can_ptr->check_for_mc_message(); 
  
    // react to message 
    if (msg) {
      print_can_message(*console, *msg);
      can_ptr->process_can_message(*msg, servo_ptr); 
      hal::print<64>(*console, "Action: %x \n", servo_ptr->get_active_action());
      new_action = true; 
    }

    // continue action 
    if(servo_ptr->get_active_action() != 0) {
      if (delay_counter >= 6) {
        delay_counter = 0;
        can_ptr->periodic_action(servo_ptr->get_active_action(), servo_ptr); 
      }
      servo_ptr->periodic_action(new_action); 
    }
    
    new_action = false; 
    delay_counter++; 
    hal::delay(*clock, 50ms); 


  }
  
}
}  // namespace sjsu::perseus