#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <mission_control_manager.hpp>

namespace sjsu::hub {
int16_axis round_clamp_int16(float x, float y, float z)
{
  auto round_and_cast = [](float f) {
    constexpr int16_t int16_min = std::numeric_limits<std::int16_t>::min();
    constexpr int16_t int16_max = std::numeric_limits<std::int16_t>::max();
    return static_cast<int16_t>(
      std::clamp<long>(lroundf(f), int16_min, int16_max));
  };
  return int16_axis{ .x = round_and_cast(x),
                     .y = round_and_cast(y),
                     .z = round_and_cast(z) };
}
}  // namespace sjsu::hub
