
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

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

// void process_can_message(hal)
enum arm_addresses : hal::u16
{
  home_address = 0x111,
  arm_address = 0x112,
  end_effector = 0x115
};


void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  std::array<hal::can_message, 32> buffer_storage;  // not sure if 32 is too big
  std::span<hal::can_message> receive_buffer(buffer_storage);
  auto can_transceiver = resources::can_transceiver(receive_buffer);
  auto can_bus_manager = resources::can_bus_manager();

  auto arm_servos = resources::arm_servos(can_transceiver);
  auto can_finders = resources::can_finders(can_transceiver,
                                            arm_addresses::home_address,
                                            arm_addresses::arm_address,
                                            arm_addresses::end_effector);

  auto console = resources::console();

  // starts homing or waits fro homing command
  // IMP: elbow will start drooping once homed because gravity. Immediately send
  // desired position as 0 once homed, and use feedforward + PID to stay at
  // homed position
  // on can command receive send appropriate parsed data to respective motors

  // filter messages this module sees? arm and drive are separate can buses so
  // it should be fine
  can_bus_manager->baud_rate(1.0_MHz);
  while (true) {
    auto optional_home_message = can_finders.home_finder->find();
    auto optional_arm_message = can_finders.arm_finder->find();
    auto optional_endeffector_message = can_finders.endeffector_finder->find();
    if (optional_home_message) {
      hal::print(*console, "Received homing command");
      print_can_message(*console, *optional_home_message);
      // process_can_message(*optional_message, target, current);
      
      // can_finder->transceiver().send()
    }
    if (optional_arm_message) {
      hal::print(*console, "Received arm movement command");
      // make some sort of arm class that converts angles provided by mission
      // control and sends those to
      // arm servos at some amount of max speed
    }
    if (optional_endeffector_message) {
      hal::print(*console, "Received end effector command");
    }
  }
}
}  // namespace sjsu::arm
