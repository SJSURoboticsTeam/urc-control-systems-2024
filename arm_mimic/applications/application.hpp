#pragma once

#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

namespace sjsu::arm_mimic {
struct application_framework
{
  hal::serial* terminal;
  hal::i2c* i2c2;
  hal::steady_clock* steady_clock;
  hal::callback<void()> reset;
};

application_framework initialize_platform();
void application(application_framework& p_framework);
}  // namespace sjsu::arm_mimic