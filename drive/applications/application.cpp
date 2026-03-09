#include <drivetrain_math.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>


namespace sjsu::drive {
void application()
{
  // resources::reset();
  // each loop:
  // -if stop message stop then stop drive
  // -if respond to heartbeat
  // -if homing stop drive and run homing sequence (make interuptable by MC to
  // cancel) -else update target chassis value if needed run periodic to keep
  // drivetrain running smoothly -return any readings requested by MC
}
}  // namespace sjsu::drive
