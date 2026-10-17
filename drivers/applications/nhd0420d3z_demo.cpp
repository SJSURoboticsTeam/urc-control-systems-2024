#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{
  auto led = resources::status_led();
  auto clock = resources::clock();

  auto console = resources::console();

  auto p_i2c = resources::i2c(0.5f, hal::i2c::settings{ .clock_rate = 10'000 });

  hal::print(*console, "init i2c\n");

  auto display = nhd0420d3z(*p_i2c);

  constexpr auto delay = 500ms;
  constexpr auto delay_short = 250ms;

  display.power(true);
  hal::delay(*clock, delay);

  display.set_backlight_brightness(8);
  hal::delay(*clock, delay_short);
  display.set_display_contrast(50);
  hal::delay(*clock, delay_short);
  display.display_message("Hello world!\nF");
  hal::delay(*clock, delay);

  display.backspace();
  hal::delay(*clock, delay_short);
  display.backspace();
  hal::delay(*clock, delay_short);
  display.backspace();
  hal::delay(*clock, delay_short);
  display.backspace();
  hal::delay(*clock, delay_short);
  display.backspace();
  hal::delay(*clock, delay_short);
  display.write_char('a');
  hal::delay(*clock, delay_short);
  display.write_char('b');
  hal::delay(*clock, delay_short);
  display.write_char('c');
  hal::delay(*clock, delay_short);
  display.write_char('d');
  hal::delay(*clock, delay_short);
  display.write_char('e');
  hal::delay(*clock, delay);

  display.home_cursor();
  display.write_char('h');
  hal::delay(*clock, delay);

  display.display_message("123456789ABCDEFGHIJK123456789ABCDEFGHIJK123456789ABCDEFGHIJK123456789ABCDEFGHIJK");
  hal::delay(*clock, delay);

  display.display_message("foo");
  hal::delay(*clock, delay_short);
  display.underline_cursor(true);
  hal::delay(*clock, delay);

  display.blinking_cursor(true);
  hal::delay(*clock, delay);

  display.display_message("123456789ABCDEFGHIJK123456789ABCDEFGHIJK123456789ABCDEFGHIJK123456789ABCDEFGHIJK");
  hal::delay(*clock, delay_short);
  display.rotate_left();
  hal::delay(*clock, delay_short);
  display.rotate_left();
  hal::delay(*clock, delay_short);
  display.rotate_left();
  hal::delay(*clock, delay_short);
  display.rotate_right();
  hal::delay(*clock, delay);

  hal::print(*console, "done\n");
}
}  // namespace sjsu::drivers
