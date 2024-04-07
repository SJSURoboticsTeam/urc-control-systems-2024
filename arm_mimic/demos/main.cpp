#include <libhal/error.hpp>

#include "hardware_map.hpp"

hardware_map_t hardware_map{};

int main()
{
  try {
    hardware_map = initialize_platform();
  } catch (...) {
    hal::halt();
  }

  application(hardware_map);

  return 0;
}