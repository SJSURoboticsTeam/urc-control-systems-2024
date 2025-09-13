
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/steady_clock.hpp>
#include <system_error>

#include "../hardware_map.hpp"

namespace sjsu::arm {

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  std::array<hal::can_message, 32> buffer_storage;  // not sure if 32 is too big
  std::span<hal::can_message> receive_buffer(buffer_storage);
  auto can_transceiver = resources::can_transceiver(receive_buffer);
  auto can_bus_manager = resources::can_bus_manager();
  auto track_servo = resources::track_servo(can_transceiver);
  auto shoulder_servo = resources::shoulder_servo(can_transceiver);
  auto elbow_servo = resources::elbow_servo(can_transceiver);
  auto wrist_roll_servo = resources::wrist_roll_servo(can_transceiver);
  auto wrist_pitch_servo = resources::wrist_pitch_servo(can_transceiver);
  auto console = resources::console();

  // starts homing or waits fro homing command
  // IMP: elbow will start drooping once homed because gravity. Immediately send
  // desired position as 0 once homed, and use feedforward + PID to stay at
  // homed position
  // on can command receive send appropriate parsed data to respective motors

  // filter messages this module sees? arm and drive are separate can buses so
  // it should be fine
  can_bus_manager->baud_rate(1.0_MHz);
  hal::u32 receive_cursor = 0;
  while (true) {
    // forever can loop
    auto const buffer = can_transceiver->receive_buffer();
    auto cursor = can_transceiver->receive_cursor();
    for (; receive_cursor != cursor;
         receive_cursor = (receive_cursor + 1) % buffer.size()) {
      print_can_message(*console, buffer[receive_cursor]);
      // process can message.
      // if homing send homing commands to servos
      // else send track servo position and speed requirements
      cursor = can_transceiver->receive_cursor();
    }
  }
}
}  // namespace sjsu::arm
