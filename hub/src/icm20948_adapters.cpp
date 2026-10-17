#include <icm20948_adapters.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/gyroscope.hpp>
#include <libhal/magnetometer.hpp>

namespace sjsu::hub {

icm20948_accelerometer::icm20948_accelerometer(
  hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
  : m_icm(p_icm)
{
}

hal::accelerometer::read_t icm20948_accelerometer::driver_read()
{
  auto readings = m_icm->read_acceleration();
  return { .x = readings.x, .y = readings.y, .z = readings.z };
}

icm20948_gyroscope::icm20948_gyroscope(
  hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
  : m_icm(p_icm)
{
}

hal::gyroscope::read_t icm20948_gyroscope::driver_read()
{
  auto readings = m_icm->read_gyroscope();
  return { .x = readings.x, .y = readings.y, .z = readings.z };
}

icm20948_magnetometer::icm20948_magnetometer(
  hal::v5::strong_ptr<hal::sensor::icm20948> p_icm)
  : m_icm(p_icm)
{
  m_icm->init_mag();
}

hal::magnetometer::read_t icm20948_magnetometer::driver_read()
{
  auto readings = m_icm->read_magnetometer();
  return { .x = readings.x, .y = readings.y, .z = readings.z };
}

}  // namespace sjsu::hub
