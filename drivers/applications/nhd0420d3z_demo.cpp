#include <array>
#include <cstdio>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>
#include <resource_list>
#include <string_view>

using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{

  auto clock = resources::clock();
  auto console = resources::console();
  hal::print<1024>(*console, "Before I2C\n");
  auto i2c = resources::i2c();
  hal::print<1024>(*console, "After I2C\n");
  hal::print<1024>(*console, "Before LCD\n");
  auto display = hal::v5::make_strong_ptr<sjsu::drivers::nhd0420d3z>(
    resources::driver_allocator(), *i2c);
  hal::print<1024>(*console, "After LCD\n");

  hal::delay(*clock, 1ms);
  display->power(true);

  // display.set_cursor_position(0,0);
  hal::delay(*clock, 1ms);

  // set contrast
  display->send_prefix();
  display->send_data(50);
  hal::delay(*clock, 1ms);

  // display i2c adress
  // display->send_prefix();
  // display->send_data(0x72);

  // display->write_char('I');
  hal::delay(*clock, 1s);
  // std::array<hal::byte, 2> command = { 0xFE, 0x72 };
  // hal::write(i2c, 0x28, command);

  // constexpr std::string_view demoMessage = "d[%d]\nx[%x]\nf[%f]";
  constexpr int buffer_size = 256;
  // display->display_message("test? worky worky?");
  hal::print<1024>(*console, "test? worky worky?\n");

  std::array<hal::byte, buffer_size> printMessage;
  // static_cast<std::string_view>(printMessage);
  // std::snprintf(reinterpret_cast<char*>(printMessage.begin()),
  //               printMessage.size(),
  //               demoMessage.begin(),
  //               31,
  //               31,
  //               31.31f);

  // display->display_message(std::string_view(
  //   reinterpret_cast<char*>(printMessage.begin()), printMessage.size()));
  // hal::print(*console, printMessage);

  while (true) {
    printMessage = hal::read<buffer_size>(*console, hal::never_timeout());
    display->display_message(std::string_view(
      reinterpret_cast<char*>(printMessage.begin()), printMessage.size()));
    hal::print(*console, printMessage);
  }
}
}  // namespace sjsu::drivers