
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/steady_clock.hpp>

#include "../hardware_map.hpp"

namespace sjsu::arm {
void application()
{
  using namespace std::chrono_literals;

  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();

  hal::print(*console, "Starting Application!\n");
  hal::print(*console, "Will reset after ~10 seconds\n");

  for (int i = 0; i < 10; i++) {
    // Print message
    hal::print(*console, "Hello, World\n");

    // Toggle LED
    led->level(true);
    hal::delay(*clock, 500ms);

    led->level(false);
    hal::delay(*clock, 500ms);
  }

  hal::print(*console, "Resetting!\n");
  hal::delay(*clock, 100ms);
}
}  // namespace sjsu::arm
