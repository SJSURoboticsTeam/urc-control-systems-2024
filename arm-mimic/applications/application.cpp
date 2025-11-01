

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

#include "./application.hpp"

namespace sjsu::mimic {
void application(hardware_map_t& hardware_map)
{
  using namespace std::chrono_literals;

  auto& led = *hardware_map.led;
  auto& clock = *hardware_map.clock;
  auto& console = *hardware_map.console;
  // rc_servoJa
  // pwm

  hal::print(console, "Starting Application!\n");
  hal::print(console, "Will reset after ~10 seconds\n");

  for (int i = 0; i < FEEDBACK_SERVOS; i++) {
    // Print message
    hal::print(console, "Hello, World\n");

    // Toggle LED
    led.level(true);
    hal::delay(clock, 500ms);

    led.level(false);
    hal::delay(clock, 500ms);
  }

  hal::print(console, "Resetting!\n");
  hal::delay(clock, 100ms);
  hardware_map.reset();
}
}  // namespace sjsu::arm