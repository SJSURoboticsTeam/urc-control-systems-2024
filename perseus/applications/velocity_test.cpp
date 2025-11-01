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

using namespace std::chrono_literals;
namespace sjsu::perseus {

enum servo_address : hal::u16
{
  track_servo = 0x120,
  shoulder_servo = 0x121,
  elbow_servo = 0x122,
  wrist_pitch = 0x123,
  wrist_roll = 0x124,
  clamp = 0x125
};

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
  
  bldc_perseus servo(h_bridge, encoder);
  hal::print(*console, "BLDC Servo created...\n");

  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  
  hal::print(*console, "BLDC Servo initialized...\n");
  int time = 0;
  float prev_reading = 0;
  while (true) {
    servo.m_h_bridge->power(-0.3);
    // auto read = servo.m_encoder->read().angle;
    // hal::print<128>(*console, "Encoder reaxding: %f\n", read);

    auto reading = servo.get_current_position();
    hal::print<128>(*console, "Encoder reading: %.2f\n", reading);
    if (time % 1000 == 0) {
      hal::print<128>(
        *console, "Velocity (degrees/second): %.2f\n", reading - prev_reading);
      prev_reading = reading;
      
    }
    hal::delay(*clock, 100ms);
    time += 100;

  } 
}
}  // namespace sjsu::perseus