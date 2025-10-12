// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../hardware_map.hpp"
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
namespace sjsu::drivers {
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
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_idf = resources::can_identifier_filter();
  
  // Change the CAN baudrate here.
  static constexpr auto baudrate = 1.0_MHz;

  can_bus_manager->baud_rate(baudrate);

  // hal::u32 _receive_cursor = 0;
  hal::can_message_finder reader(*can_transceiver, 0x110);
  can_idf->allow(0x110);

  while (true) {
    using namespace std::chrono_literals;
    hal::can_message standard_message {
      .id=0x112,
      .extended=false,
      .remote_request=false,
      .length = 8,
      .payload = {
        0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF,
      },
    };


    can_transceiver->send(standard_message);

    hal::delay(*clock, 1s);

    std::optional<hal::can_message> found_message = reader.find();
    if (found_message) {
      print_can_message(*console, *found_message);
    } else {
      // Message has not not been received yet
    }
    hal::print(*console, "Printing done.\n\n");
  }
}
}  // namespace sjsu::drivers
