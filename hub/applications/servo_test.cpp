#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <resource_list.hpp>

namespace sjsu::hub {

using namespace std::chrono_literals;
using namespace hal::literals;

// Simple servo sweep test. No CAN, no IMU, no PID.
// If servos don't move here, it's a hardware/wiring issue.
void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  hal::print(*console, "=== SERVO SWEEP TEST ===\n");

  auto x_servo = resources::yaw_servo();
  auto y_servo = resources::pitch_servo();
  
  hal::print(*console, "starting sweep in 2 seconds...\n");
  hal::delay(*clock, 2000ms);

  hal::print(*console, "=== SWEEPING ===\n");
  while (true) {
    hal::print(*console, "yaw -> 0, pitch -> 30 (both min)\n");
    x_servo->position(0.0f);
    y_servo->position(30.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "yaw -> 90, pitch -> 90 (both center)\n");
    x_servo->position(90.0f);
    y_servo->position(90.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "yaw -> 180, pitch -> 150 (both max)\n");
    x_servo->position(180.0f);
    y_servo->position(150.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "yaw -> 90, pitch -> 90 (both center)\n");
    x_servo->position(90.0f);
    y_servo->position(90.0f);
    hal::delay(*clock, 1500ms);

    hal::print(*console, "--- loop complete, repeating ---\n");
  }
}
}  // namespace sjsu::hub
