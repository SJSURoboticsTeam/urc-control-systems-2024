#pragma once

#include "steering_module.hpp"
#include <libhal/units.hpp>
#include <span>
namespace sjsu::drive {

void home(std::span<steering_module> legs);

hal::degrees small_magnets_theta = 90;
hal::degrees big_magnets_theta = 360 - small_magnets_theta;


}  // namespace sjsu::drive