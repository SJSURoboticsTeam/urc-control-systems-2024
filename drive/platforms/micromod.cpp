#include <libhal-micromod/micromod.hpp>

#include "../applications/application.hpp"

namespace sjsu::drive {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;

  hal::micromod::v1::initialize_platform();

  return {
    .led = &hal::micromod::v1::led(), 
    .console = &hal::micromod::v1::console(hal::buffer<128>),
    .clock = &hal::micromod::v1::uptime_clock(),
    .reset = +[]() { hal::micromod::v1::reset(); },
  };
}
}
