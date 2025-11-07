// TODO this attempts to home track, shoulder and elbow without mc input


#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include "can_ids.hpp"
#include "../hardware_map.hpp"
#include "../include/home.hpp"
#include "../include/can_message_processor.hpp"

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
// response: 0x200: byte1: servo#, byte2 byte3 (desired angle)
// speed cap

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto console = resources::console();
  auto clock = resources::clock();
  auto arm_servos = resources::arm_servos(can_transceiver);
  auto can_finders = resources::can_finders(can_transceiver);
  auto filters = resources::can_filters();
  auto home = Home(arm_servos, resources::arm_home_pins());
  can_bus_manager->baud_rate(1.0_MHz);

  // need to have a way to make the servo not move BUT allow the motor to
  // move enough to counter the torque.
  bool isHomed = false;
  while (true) {
    home.home_all_joints();
  }
}
}  // namespace sjsu::arm
