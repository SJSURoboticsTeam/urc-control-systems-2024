#pragma once

#include <cstdint>

namespace sjsu::hub {

struct int16_axis
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct float_axis
{
  float x;
  float y;
  float z;
};

/**
 * @brief converts a float_axis to an int16_axis by rounding all float values
 * and clamping them to the int16 range.
 */
int16_axis round_clamp_int16(float_axis p_axis);

double rad_to_deg(double p_rad);

}  // namespace sjsu::hub
