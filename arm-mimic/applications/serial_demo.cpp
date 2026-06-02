#include <libhal-exceptions/control.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <resource_list.hpp>

#include <cstdlib>

namespace sjsu::mimic {
void application()
{
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto servo_settings = resources::rc_servo_settings();
  auto test_servo = resources::rc_servo();
  auto test_servo_feedback = resources::a0_feedback_adc();
  float tolerance = 2.0f;  // degrees

  hal::print(*console, "Testing serial!!!\n\n");

  while (true) {
    hal::print(*console, "Enter a servo angle: ");

    // TODO: buffer value is set arbitrarily right now
    std::array<char, 4> buffer{};  // initializes buffer to be 0

    // TODO: have to check for \n to wait for buffer to fill up
    std::span<hal::byte> input{ reinterpret_cast<hal::byte*>(buffer.data()),
                                buffer.size() - 1 };
    hal::read(*console, input, hal::never_timeout());
    hal::print<64>(*console, "Input degree: %s\n", buffer.data());

    // Make sure user does not blow up servo
    float deg_value = std::strtof(buffer.data(), nullptr);
    hal::degrees angle = deg_value;
    if (angle > servo_settings.max_angle || angle < servo_settings.min_angle) {
      hal::print<64>(*console, "Invalid angle. Please enter a value between %f and %f\n\n",
                 servo_settings.min_angle,
                 servo_settings.max_angle);
      continue;
    }

    // Set servo angle to user defined value
    hal::print(*console, "Setting servo angle...\n");
    test_servo->position(angle);
    hal::delay(*clock, 2000ms);

    // Map potentiometer value to 180 degrees
    std::pair servo_output_range = { 0.06, 0.89 };
    std::pair output_percent = { 0, 180 };
    float servo_pos = test_servo_feedback->read();
    float servo_percent_pos =
      hal::map(servo_pos, servo_output_range, output_percent);
    hal::print<64>(*console, "ADC: %f\n", servo_pos);
    hal::print<64>(*console, "ADC Mapped: %f\n", servo_percent_pos);
    if (std::abs(servo_percent_pos - deg_value) < tolerance) {
      hal::print(*console, "Servo angle reached!\n");
    }
    hal::print(*console, "\n");
  }
}
}  // namespace sjsu::mimic
