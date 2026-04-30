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
  auto test_servo_feedback = resources::a0_feedback_adc();

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