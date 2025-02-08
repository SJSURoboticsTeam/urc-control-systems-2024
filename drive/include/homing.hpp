#pragma once

#include "steering_module.hpp"
#include "settings.hpp"
#include <libhal/units.hpp>
#include <span>
namespace sjsu::drive {

void home(std::span<steering_module> legs, std::span<module_settings> leg_settings);

float curr_max = 10;
float curr_min = -10;


}  // namespace sjsu::drive