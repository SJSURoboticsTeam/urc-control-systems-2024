#include <libhal/error.hpp>

#include "applications/application.hpp"

sjsu::arm::hardware_map_t hardware_map{};

int main()
{
  try {
    hardware_map = sjsu::arm::initialize_platform();
  } catch (...) {
    hal::halt();
  }

  sjsu::arm::application(hardware_map);
  return 0;
}

// TODO(#129) remove this later (used in building for debug) 
extern "C" {
  _reent* _impure_ptr = nullptr;
  void __assert_func() {
  }
}