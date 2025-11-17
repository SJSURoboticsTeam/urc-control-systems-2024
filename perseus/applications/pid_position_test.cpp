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

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  hal::print(*console, "CAN message finder initialized...\n");
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");

  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  hal::print(*console, "BLDC Servo initialized...\n");
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.01,
    // .ki = 0.05,
    .ki = 0,
    .kd = 0,
  };
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_target_position(-30);

  while (true) {
    // csv output for easy graphing
    servo_ptr->update_position_noff();
    hal::delay(*clock, 10ms);

  } 
}
}  // namespace sjsu::perseus