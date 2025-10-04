#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto a_high = resources::output_pin_5();
  auto b_high = resources::output_pin_6();
  auto a_low = resources::pwm_channel_0();
  auto b_low = resources::pwm_channel_1();
  auto clock = resources::clock();
  // auto& terminal = *p_resources.terminal;
  auto m_h_bridge = h_bridge(a_low, b_low, a_high, b_high);

  while (true) {
    m_h_bridge.power(0.1);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(0.4);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(0.7);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(1);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(-0.1);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(-0.4);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(-0.7);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(-1);
    hal::delay(*clock, 2000ms);
    m_h_bridge.power(0);
    hal::delay(*clock, 2000ms);
  }
}
}  // namespace sjsu::drivers