
#include "vector2d.hpp"
#include <libhal/units.hpp>

namespace sjsu::drive {

constexpr int module_count = 4;

using meters_per_sec = float;
using meters_per_sec_per_sec = float;
using deg_per_sec = float;
using sec = float;
using radians = float;

struct swerve_module_state
{
  hal::degrees steer_angle;
  meters_per_sec propulsion_velocity;
};

struct chassis_velocities
{
  vector2d translation;
  deg_per_sec rotational_vel;
};

}  // namespace sjsu::drive