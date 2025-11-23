#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {
void application()
{
  auto terminal = perseus::resources::console();
  hal::print<128>(*terminal, "Terminal Initiated...\n");
  auto h_bridge = perseus::resources::h_bridge();
  hal::print<128>(*terminal, "H-Bridge Acquired...\n");
  auto clock = perseus::resources::clock();
  hal::print<128>(*terminal, "H-Bridge Demo Application Starting...\n");
  while (true) {
    h_bridge->power(0.1);
    hal::delay(*clock, 2000ms);
    h_bridge->power(0.4);
    hal::delay(*clock, 2000ms);
    h_bridge->power(0.7);
    hal::delay(*clock, 2000ms);
    h_bridge->power(1);
    hal::delay(*clock, 2000ms);
    h_bridge->power(-0.1);
    hal::delay(*clock, 2000ms);
    h_bridge->power(-0.4);
    hal::delay(*clock, 2000ms);
    h_bridge->power(-0.7);
    hal::delay(*clock, 2000ms);
    h_bridge->power(-1);
    hal::delay(*clock, 2000ms);
    h_bridge->power(0);
    hal::delay(*clock, 2000ms);
  }
}
}  // namespace sjsu::drivers