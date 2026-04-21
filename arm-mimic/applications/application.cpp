

// rc servo x 8
// analog
// clock
// serial

// 4 servos
//      make adc
//      
// 1 dc motor/encoder
// lights


// class to take in adc and rc servo
// read and write to feedback_servo
//      start record funxtion - an array to be recorded?

// copy conan, cmakelists, and main.cpp for things to work and hardwaremap.hpp
//      refer to hub

// main file is application

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();
 
  // pwm
  auto test_servo = resources::rc_servo();
  auto test_servo_feedback = resources::test_servo_feedback_adc_0();

  hal::print(*console, "Starting Application!\n");
  hal::print(*console, "Will reset after ~10 seconds\n");

  while (true) {
    hal::print(*console, "Set to: -90 deg\n");
    test_servo->position(-90.0);
    hal::delay(*clock, 2000ms);
    hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());

    hal::print(*console, "Set to: 0 deg\n");
    test_servo->position(0.0);
    hal::delay(*clock, 2000ms);
    hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());

    hal::print(*console, "Set to: 90 deg\n");
    test_servo->position(90.0);
    hal::delay(*clock, 2000ms);
    hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());
  }

  hal::print(*console, "Resetting!\n");
  hal::delay(*clock, 100ms);
}
}  // namespace sjsu::mimic