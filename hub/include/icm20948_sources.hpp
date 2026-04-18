#pragma once

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal/pointers.hpp>

#include <sensor_sources.hpp>

namespace sjsu::hub {

class icm20948_accel_source : public accel_source
{
public:
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

class icm20948_gyro_source : public gyro_source
{
public:
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

class icm20948_mag_source : public mag_source
{
public:
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