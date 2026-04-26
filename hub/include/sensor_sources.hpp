#pragma once

#include <libhal/pointers.hpp>

namespace sjsu::hub {

/**
 * @brief generic 3-axis sensor reading
 */
struct sensor_axis
{
  float x;
  float y;
  float z;
};

/**
 * @brief abstract interface for any accelerometer source. Implement this to
 * support different IMU hardware (ICM20948, MPU6050, etc.)
 */
class accel_source
{
public:
  virtual sensor_axis read_acceleration() = 0;
  virtual ~accel_source() = default;
};

/**
 * @brief abstract interface for any gyroscope source
 */
class gyro_source
{
public:
  virtual sensor_axis read_gyroscope() = 0;
  virtual ~gyro_source() = default;
};

/**
 * @brief abstract interface for any magnetometer source
 */
class mag_source
{
public:
  virtual sensor_axis read_magnetometer() = 0;
  virtual ~mag_source() = default;
};

}  // namespace sjsu::hub