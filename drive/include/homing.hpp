#include <cmath>
#include <cstddef>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <span>
#include "swerve_module.hpp"


namespace sjsu::drive {

void home(hal::v5::strong_ptr<std::array<hal::v5::strong_ptr<swerve_module>, 4> > legs,
           hal::v5::strong_ptr<hal::serial> terminal);

}  // namespace sjsu::drive