#pragma once

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/gyroscope.hpp>
#include <libhal/magnetometer.hpp>
#include <libhal/pointers.hpp>

namespace sjsu::hub {

/**
 * @brief ICM20948 accelerometer adapter. Wraps the ICM20948 driver to
 * implement the accel_source interface
 */
class icm20948_accelerometer : public hal::accelerometer
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_accelerometer(
    hal::v5::strong_ptr<hal::sensor::icm20948> p_icm);

private:
  virtual read_t driver_read();

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

/**
 * @brief ICM20948 gyroscope adapter. Wraps the ICM20948 driver to implement
 * the gyro_source interface
 */
class icm20948_gyroscope : public hal::gyroscope
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_gyroscope(hal::v5::strong_ptr<hal::sensor::icm20948> p_icm);

private:
  virtual read_t driver_read() override;

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

/**
 * @brief ICM20948 magnetometer adapter. Wraps the ICM20948 driver to
 * implement the mag_source interface
 */
class icm20948_magnetometer : public hal::magnetometer
{
public:
  /**
   * @param p_icm shared pointer to the ICM20948 device
   */
  explicit icm20948_magnetometer(
    hal::v5::strong_ptr<hal::sensor::icm20948> p_icm);

private:
  virtual read_t driver_read() override;

private:
  hal::v5::strong_ptr<hal::sensor::icm20948> m_icm;
};

}  // namespace sjsu::hub
