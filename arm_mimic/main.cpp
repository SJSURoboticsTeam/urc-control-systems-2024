#include <libhal/error.hpp>

#include "applications/application.hpp"

sjsu::arm_mimic::application_framework hardware_map{};

int main()
{
  try {
    hardware_map = sjsu::arm_mimic::initialize_platform();
  } catch (...) {
    hal::halt();
  }

  sjsu::arm_mimic::application(hardware_map);

  return 0;
}