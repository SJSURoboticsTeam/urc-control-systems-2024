#pragma once

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal/pointers.hpp>

#include <sensor_sources.hpp>

namespace sjsu::hub {

/**
 * @brief ICM20948 accelerometer adapter. Wraps the ICM20948 driver to
 * implement the accel_source interface
 */
class icm20948_accel_source : public accel_source
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_accel_source(
    hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
    : m_icm(p_icm)
  {
  }

  sensor_axis read_acceleration() override
  {
    auto value = m_icm->read_acceleration();
    return sensor_axis{ .x = value.x, .y = value.y, .z = value.z };
  }

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

/**
 * @brief ICM20948 gyroscope adapter. Wraps the ICM20948 driver to implement
 * the gyro_source interface
 */
class icm20948_gyro_source : public gyro_source
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_gyro_source(
    hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
    : m_icm(p_icm)
  {
  }

  sensor_axis read_gyroscope() override
  {
    auto value = m_icm->read_gyroscope();
    return sensor_axis{ .x = value.x, .y = value.y, .z = value.z };
  }

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

/**
 * @brief ICM20948 magnetometer adapter. Wraps the ICM20948 driver to
 * implement the mag_source interface
 */
class icm20948_mag_source : public mag_source
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_mag_source(
    hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
    : m_icm(p_icm)
  {
  }

  sensor_axis read_magnetometer() override
  {
    auto value = m_icm->read_magnetometer();
    return sensor_axis{ .x = value.x, .y = value.y, .z = value.z };
  }

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

}  // namespace sjsu::hub