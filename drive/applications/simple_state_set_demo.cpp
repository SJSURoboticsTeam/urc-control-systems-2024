#include <array>
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>
#include <swerve_module.hpp>

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  try {
    auto swerve_modules_ptr = resources::swerve_modules();
    auto& swerve_modules = *swerve_modules_ptr;
    // std::array swerve_modules = {resources::front_left_swerve_module()};
    hal::print(*console, "modules defined\n");
    hal::print(*console, "starting homing!\n");
    for (uint8_t i = 0; i < swerve_modules.size(); i++) {
      try {
        swerve_modules.at(i)->hard_home();
        hal::print<64>(*console, "Homed wheel: %d\n", i);
      } catch (hal::exception e) {
        hal::print<64>(*console, "Wheel throwing error %d\n", i);
        throw;
      }
    }
    for (uint8_t i = 0; i < swerve_modules.size(); i++) {
      swerve_modules.at(i)->set_target_state(swerve_module_state(0, 0));
    }
    hal::delay(*clock, 2s);
    hal::print(*console, "loop starting!\n");
    std::array module_states = {
      swerve_module_state(90, 0),
      swerve_module_state(-90, 0),
      swerve_module_state(0, 0.125),
      swerve_module_state(0, -0.125),
    };
    while (true) {
      for (auto const& sm : module_states) {
        for (uint8_t i = 0; i < swerve_modules.size(); i++) {
          swerve_modules.at(i)->set_target_state(sm);
        }
        hal::print<128>(
          *console, "(%f,%f)\n", sm.steer_angle, sm.propulsion_velocity);
        hal::delay(*clock, 3s);
      }
    }
  } catch (hal::exception e) {
    hal::print<128>(*console, "Exception code %d\n", e.error_code());
  }
}
}  // namespace sjsu::drive
