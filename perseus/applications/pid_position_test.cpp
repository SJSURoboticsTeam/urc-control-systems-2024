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
    .kp = 0.5,
    .ki = 0.05,
    .kd = 0,
  };
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_target_position(-30);

  while (true) {

    // // print velocity 
    // hal::u64 curr_time = clock->uptime(); 
    // float curr_angle = servo.get_current_position();
    // hal::u64 dt = curr_time - servo.m_last_clock_check; 
    // float da = curr_angle - servo.m_prev_encoder_value; 
    // float da_dt = 10 * da / static_cast<float>(dt); 
    // hal::print<128>(*console, "VELOCITY: %.2f\n", da_dt);

    // servo.m_last_clock_check = curr_time;
    // servo.m_prev_encoder_value = curr_angle;
    
    servo_ptr->update_position_noff();
    hal::print<128>(*console, "Current Position: %.2f\n", servo_ptr->get_current_position());

    hal::delay(*clock, 10ms);

  } 
}
}  // namespace sjsu::perseus