#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>

#include "application.hpp"
namespace sjsu::science {

hal::status application(application_framework& p_framework)
{
  using namespace std::literals;

  auto& clock = *p_framework.steady_clock;
  auto& console = *p_framework.terminal;

  while (true) {
    // Print message
    hal::print(console, "Hello, World\n");

  }
}
}  // namespace sjsu::science
