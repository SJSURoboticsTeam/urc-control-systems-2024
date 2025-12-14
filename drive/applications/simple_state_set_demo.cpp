#include <swerve_module.hpp>
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
        (*swerve_modules)[i]->hard_home();
        hal::print<64>(*console, "Homed wheel: %d\n", i);
      } catch (hal::exception e) {
        hal::print<64>(*console, "Wheel throwing error %d\n", i);
        throw e;
      }
    }
    for (int i = 0; i < module_count; i++) {
      (*swerve_modules)[i]->set_target_state(swerve_module_state(0,0));
    }
    hal::delay(*clock, 10s);
    while (true) {
      for (int i = 0; i < module_count; i++) {
        (*swerve_modules)[i]->set_target_state(swerve_module_state(90, 0));
      }
      hal::print(*console, "(90,0)\n");
      hal::delay(*clock, 10s);
      for (int i = 0; i < module_count; i++) {
        (*swerve_modules)[i]->set_target_state(swerve_module_state(-90, 0));
      }
      hal::print(*console, "(-90,0)\n");
      hal::delay(*clock, 10s);
      for (int i = 0; i < module_count; i++) {
        (*swerve_modules)[i]->set_target_state(swerve_module_state(0, 0.125));
      }
      hal::print(*console, "(0, 0.125)\n");
      hal::delay(*clock, 10s);
      for (int i = 0; i < module_count; i++) {
        (*swerve_modules)[i]->set_target_state(swerve_module_state(0, -0.125));
      }
      hal::print(*console, "(0, -0.125)\n");
      hal::delay(*clock, 10s);
    }
  } catch (hal::exception e) {
    hal::print<128>(*console, "Exception code %d\n", e.error_code());
  }
}
}  // namespace sjsu::drive
