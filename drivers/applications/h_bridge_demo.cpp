#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto terminal = resources::console();
  hal::print(*terminal, "Hey beginnning application\n");


  auto a_high = resources::pwm_channel_0();
  hal::print(*terminal, "Getting a high\n");

  auto b_high = resources::pwm_channel_1();
  auto pwm_freq = resources::pwm_frequency();

  hal::print(*terminal, "Getting b high\n");

  auto a_low = resources::a_low();
  hal::print(*terminal, "pwm channel 0\n");

  auto b_low = resources::b_low();

  hal::print(*terminal, "pwm channel 1\n");

  auto clock = resources::clock();

  hal::print(*terminal, "clock\n");

  auto m_h_bridge = h_bridge(a_high, b_high, a_low, b_low);

  hal::print(*terminal, "h_brid\n");

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