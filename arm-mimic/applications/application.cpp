

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

  // auto& led = *hardware_map.led;
  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();
 
  // rc_servoJa
  // pwm

  hal::print(*console, "Starting Application!\n");
  hal::print(*console, "Will reset after ~10 seconds\n");

  // for (int i = 0; i < FEEDBACK_SERVOS; i++) {
  while (true) {
    // Print message
    hal::print(*console, "Hello, World\n");

    // Toggle LED
    led->level(true);
    hal::delay(*clock, 500ms);

    led->level(false);
    hal::delay(*clock, 500ms);
  }

  // hal::print(*console, "Resetting!\n");
  // hal::delay(*clock, 100ms);
}
}  // namespace sjsu::mimic