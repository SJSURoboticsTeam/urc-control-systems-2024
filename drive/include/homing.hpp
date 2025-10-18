#include <cmath>
#include <cstddef>
#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <span>
#include "swerve_module.hpp"


namespace sjsu::drive {

void home(std::span<swerve_module> legs,
        //   hal::can_transceiver& can,
          hal::steady_clock& clock,
          hal::serial& terminal);


}  // namespace sjsu::drive