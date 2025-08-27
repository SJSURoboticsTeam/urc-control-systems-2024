#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {
void application(application_framework& p_resources)
{

  auto& a_high = *p_resources.out_pin0;
  auto& b_high = *p_resources.out_pin1;
  auto& a_low = *p_resources.pwm0;
  auto& b_low = *p_resources.pwm1;
  auto& clock = *p_resources.steady_clock;
  // auto& terminal = *p_resources.terminal;
  auto m_h_bridge = h_bridge(a_low, b_low, a_high, b_high);

  while (true) {
    m_h_bridge.power(0.1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(0.4);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(0.7);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.1);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.4);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-0.7);
    hal::delay(clock, 2000ms);
    m_h_bridge.power(-1);
    hal::delay(clock, 2000ms);

    m_h_bridge.power(0);
    hal::delay(clock, 2000ms);
  }
}
}  // namespace sjsu::drivers