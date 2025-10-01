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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include "../hardware_map.hpp"
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
  hal::print(*console, "console init!\n");

  std::array<hal::can_message, 32> buffer_storage;  // not sure if 32 is too big
  std::span<hal::can_message> receive_buffer(buffer_storage);
  auto can_transceiver = resources::can_transceiver(receive_buffer);
  hal::print(*console, "transceiver init!\n");

  auto can_bus_manager = resources::can_bus_manager();
  hal::print(*console, "bus manager init!\n");

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 1.0_MHz;

  hal::print(*console, "Starting CAN demo!\n");

  can_bus_manager->baud_rate(baudrate);
  hal::print(*console, "baud rate set.!\n");

  // hal::u32 receive_cursor = 0;

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

    hal::print(*console, "Sending payload(s)...\n");

    can_transceiver->send(standard_message);
    hal::print(*console, "m1 sent!\n");

    hal::delay(*clock, 10s);

    // hal::print(*console,
    //            "Printing received messages stored in circular buffer...\n");
    // auto const buffer = can_transceiver->receive_buffer();
    // auto cursor = can_transceiver->receive_cursor();
    // for (; receive_cursor != cursor;
    //      receive_cursor = (receive_cursor + 1) % buffer.size()) {
    //   print_can_message(*console, buffer[receive_cursor]);
    //   cursor = can_transceiver->receive_cursor();
    // }

    // hal::print(*console, "Printing done.\n\n");
  }
}
}  // namespace sjsu::drivers
