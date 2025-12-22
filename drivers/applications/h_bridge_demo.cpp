#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::drivers {
void application()
{
  auto terminal = resources::console();
  auto a_high = resources::pwm_channel_0();
  auto b_high = resources::pwm_channel_1();
  // output pins 1 - 3 don't work
  auto a_low = resources::output_pin_0();
  auto b_low = resources::output_pin_4();

  auto clock = resources::clock();

  auto m_h_bridge = h_bridge({ a_high, a_low }, { b_high, b_low });
  hal::print(*terminal, "Starting H-Bridge Demo\n");
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