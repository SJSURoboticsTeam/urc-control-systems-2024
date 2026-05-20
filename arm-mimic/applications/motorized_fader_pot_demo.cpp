#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{

  auto clock = resources::clock();
  auto console = resources::console();

  // pwm
  auto test_servo = resources::rc_servo();
  auto a0 = resources::a0_feedback_adc();
  auto a1 = resources::a1_adc();

  hal::print(*console, "Starting Feedback Servo Demo!\n");

}
}  // namespace sjsu::mimic