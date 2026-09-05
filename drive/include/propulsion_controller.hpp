#pragma once
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

namespace sjsu::drive {

class propulsion_controller
{
public:
  virtual void stop() = 0;
  virtual void set_target_velocity(hal::rpm p_velocity) = 0;
  virtual hal::rpm get_target_velocity() = 0;
  virtual hal::rpm get_actual_velocity() = 0;
};
}  // namespace sjsu::drive
