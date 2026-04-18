#pragma once

#include <libhal/pointers.hpp>

namespace sjsu::hub {

struct sensor_axis
{
  float x;
  float y;
  float z;
};

class accel_source
{
public:
  virtual sensor_axis read_acceleration() = 0;
  virtual ~accel_source() = default;
};

class gyro_source
{
public:
  virtual sensor_axis read_gyroscope() = 0;
  virtual ~gyro_source() = default;
};

class mag_source
{
public:
  virtual sensor_axis read_magnetometer() = 0;
  virtual ~mag_source() = default;
};

}  // namespace sjsu::hub