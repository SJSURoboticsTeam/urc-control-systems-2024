#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto h_bridge = perseus::resources::h_bridge();
  auto clock = perseus::resources::clock();

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