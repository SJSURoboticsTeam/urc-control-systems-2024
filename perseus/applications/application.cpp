// copied from drivers/applications/h_bridge_demo.cpp

#include <h_bridge.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include "../hardware_map.hpp"

using namespace std::chrono_literals;
namespace sjsu::perseus {

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

enum class action : hal::byte
{
  actuate_torque = 0x10,
  clamp_speed = 0x11,
  actuate_position = 0x12,
  read_position = 0x20,
  read_velocity = 0x21,
};

struct status  // this struct will be both target and current
{
  hal::u16 position;
  hal::u16 velocity;
};



void process_can_message(hal::can_message const& p_message,
                         status& target_status,
                         status const& current_status)
{
  switch (static_cast<action>(p_message.payload[0])) {
    case action::read_position:
      target_status break;
    case action::actuate_position:
      break;
    case action::clamp_speed:
      break;
    case action::actuate_torque:
      break;
    case action::read_velocity:
      break;
  }
}

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto h_bridge = drivers::h_bridge(resources::pwm_channel_0(),
                                    resources::pwm_channel_1(),
                                    resources::output_pin_0(),
                                    resources::output_pin_1());
  std::array<hal::can_message, 32> buffer_storage;  // not sure if 32 is too big
  std::span<hal::can_message> receive_buffer(buffer_storage);
  auto can_transceiver = resources::can_transceiver(receive_buffer);
  auto bus_manager = resources::can_bus_manager();
  auto console = resources::console();
  hal::u16 servo_address =
    0x120;  // try to input this somehow idk but this needs to change depending
            // on device flashing
  auto can_finder = resources::can_finder(can_transceiver, servo_address);
  bus_manager->baud_rate(1.0_MHz);

  // do nothing on startup
  status current = { .position = 0, .velocity = 0 };
  status target = { .position = 0, .velocity = 0 };
  while (true) {
    // forever can loop
    auto optional_message = can_finder->find();
    if (optional_message) {
      print_can_message(*console, *optional_message);
      process_can_message(*optional_message, target, current);
      //
      // can_finder->transceiver().send()
    }
  }
}
}  // namespace sjsu::perseus