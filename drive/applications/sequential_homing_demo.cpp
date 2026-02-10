#include "swerve_module.hpp"
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  try {
    auto swerve_modules = resources::swerve_modules();
    hal::print(*console, "modules defined\n");
    hal::print(*console, "starting homing!\n");
    for (int i = 0; i < module_count; i++) {
      try {
        (*swerve_modules)[i]->async_home_begin();
      } catch (hal::exception e) {
        hal::print<64>(*console, "Wheel home throwing error %d\n", i);
        throw e;
      }
    }
    while (true) {
      bool homed = true;
      for (int i = 0; i < module_count; i++) {
        try {
          switch (swerve_modules->at(i)->async_home_poll()) {
            case async_home_status::in_progress:
              homed = false;
              break;
            case async_home_status::completed:
              hal::print<64>(*console, "Wheel homed %d\n", i);
              break;
            case async_home_status::inactive:
              break;
          }
          // swerve_modules->at(i)->set_target_state(swerve_module_state(0, 0));
        } catch (hal::exception e) {
          hal::print<64>(*console, "Wheel home poll throwing error %d\n", i);
          throw e;
        }
      }
      if (homed) {
        break;
      }
      hal::delay(*clock, 250ms);  // supposedly safe
    }
  } catch (hal::exception e) {
    hal::print<128>(*console, "Exception code %d\n", e.error_code());
  }
}
}  // namespace sjsu::drive
