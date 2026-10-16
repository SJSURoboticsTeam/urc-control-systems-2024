#pragma once

#include <mission_control_manager.hpp>

namespace sjsu::hub {

struct float_axis
{
  float x;
  float y;
  float z;
};

int16_axis round_clamp_int16(float_axis p_axis);

double rad_to_deg(double p_rad);

}  // namespace sjsu::hub
