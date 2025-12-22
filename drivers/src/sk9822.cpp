#include <sk9822.hpp>
#include <array>
#include <libhal-util/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void light_strip_util::set_all(light_strip_view lights,
                               hal::byte const r,
                               hal::byte const g,
                               hal::byte const b,
                               hal::byte const brightness)
{
  rgb_brightness setting;
  setting.r = r;
  setting.g = g;
  setting.b = b;
  setting.brightness = brightness;
  for (auto i = lights.begin(); i != lights.end(); i++) {
    *i = setting;
  }
}

void light_strip_util::set_all(light_strip_view lights,
                               rgb_brightness const value)
{
  for (auto i = lights.begin(); i != lights.end(); i++) {
    *i = value;
  }
}

sk9822::sk9822(hal::v5::optional_ptr<hal::output_pin> p_clock_pin,
               hal::v5::optional_ptr<hal::output_pin> p_data_pin,
               hal::v5::optional_ptr<hal::steady_clock> p_clock)
{
  clock_pin = p_clock_pin;
  data_pin = p_data_pin;
  clock = p_clock;
}

void sk9822::update(light_strip_view lights)
{
  // Start Frame
  send_byte(0x00);
  send_byte(0x00);
  send_byte(0x00);
  send_byte(0x00);

  for (auto i = lights.begin(); i != lights.end(); i++) {
    send_byte((*i).brightness | 0b11100000);
    // send_byte((*i).brightness & 0x00011111);
    send_byte((*i).b);
    send_byte((*i).g);
    send_byte((*i).r);
  }

  // End Frame
  send_byte(0xff);
  send_byte(0xff);
  send_byte(0xff);
  send_byte(0xff);
}

void sk9822::send_byte(hal::byte data)
{
  for (int i = 7; i >0; i--) {
    if (data & (1 << i)) {
      (*data_pin).level(true);
    } else {
      (*data_pin).level(false);
    }
    hal::delay(*clock, half_period);
    (*clock_pin).level(true);
    hal::delay(*clock, period);
    (*clock_pin).level(false);
    hal::delay(*clock, half_period);
  }
}

};  // namespace sjsu::drivers