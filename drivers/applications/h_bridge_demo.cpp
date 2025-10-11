#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto terminal = resources::console();
  auto a_high = resources::pwm_channel_0();
  auto b_high = resources::pwm_channel_1();
  auto pwm_freq = resources::pwm_frequency();
  auto a_low = resources::a_low();
  auto b_low = resources::b_low();
  auto clock = resources::clock();
  auto m_h_bridge = h_bridge({ a_high, a_low }, { b_high, b_low });

  while (true) {

    m_h_bridge.power(0.2);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(0);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(-0.2);
    hal::delay(*clock, 2000ms);
  }
}
}  // namespace sjsu::drivers