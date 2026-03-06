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

#include "perseus_can_demo.cpp"


using namespace std::chrono_literals;
namespace sjsu::perseus {

void application()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // gen
  auto clock = resources::clock();
  auto console = resources::console();
  // CHANGE SERVO
  // track 
  constexpr can_perseus::servo_address allowed_id = can_perseus::track_servo; 
  // pid
  bldc_perseus::PID_settings pid_settings = {
    .kp = 0.04, 
    .ki = 0.00, 
    .kd = 0.00,
  };
  // servo_values 
  bldc_perseus::servo_values servo_values = {
    .gear_ratio = 16915.5, // 751.8 * 1 / 2 * 360 / 8 (for mm) 
    .angle_offset = 0, 
    .fight_gravity = 0, 
    .high_clamped_value = 0.3, 
    .low_clamped_value = -0.3 
  }; 
  // bldc
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  servo_ptr->update_pid_position(pid_settings);
  servo_ptr->set_servo_values(servo_values); 
  servo_ptr->set_actual_position();
  // can
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  static constexpr auto baudrate = 1_MHz;
  can_perseus servo_can(allowed_id, baudrate, can_transceiver, can_bus_manager, can_interrupt, can_id_filter); 
  auto can_ptr = hal::v5::make_strong_ptr<decltype(servo_can)>(resources::driver_allocator(), std::move(servo_can));
  hal::print(*console, "Servo can creature setup...\n");
  
  // Starting 
  hal::print(*console, "Starting Track\n");
  can_application(servo_ptr, can_ptr); 
  
}
}  // namespace sjsu::perseus