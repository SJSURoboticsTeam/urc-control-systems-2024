
#include <libhal/units.hpp>
#include <vector2d.hpp>


namespace sjsu::drive {

constexpr int module_count = 4;

using meters_per_sec = float;
using meters_per_sec_per_sec = float;
using deg_per_sec = float;
using sec = float;
using radians = float;

struct swerve_module_state
{
  hal::degrees steer_angle = 0.0f;
  meters_per_sec propulsion_velocity = 0.0f;
  constexpr bool operator==(swerve_module_state const& b) const
  {
    return steer_angle == b.steer_angle && propulsion_velocity == b.propulsion_velocity;
  }
  constexpr bool operator!=(swerve_module_state const& b) const
  {
    return !(*this == b);
  }
};

struct chassis_velocities
{
  vector2d translation;
  deg_per_sec rotational_vel = 0.0f;
};

}  // namespace sjsu::drive