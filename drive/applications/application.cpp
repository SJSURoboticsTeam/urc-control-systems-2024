
#include "./application.hpp"
#include "../include/homing.hpp"
#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

namespace sjsu::drive {
void application()
{
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto transceiver = resources::can_transceiver();
  try {
    auto swerve_modules =
      resources::swerve_modules(console, clock, transceiver);
    hal::print(*console, "modules defined\n");
    hal::print(*console, "starting homing!\n");
    home(swerve_modules, console);
  } catch (hal::exception e) {
    hal::print<128>(*console, "Exception code %d\n", e.error_code());
  }
  
  // resources::reset();
  // each loop:
  // -if stop message stop then stop drive
  // -if respond to heartbeat
  // -if homing stop drive and run homing sequence (make interuptable by MC to
  // cancel) -else update target chassis value if needed run periodic to keep
  // drivetrain running smoothly -return any readings requested by MC
}
}  // namespace sjsu::drive
