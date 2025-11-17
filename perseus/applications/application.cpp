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