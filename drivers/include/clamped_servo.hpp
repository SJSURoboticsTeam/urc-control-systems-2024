#pragma once

#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

/**
 * @brief clamped_servo simply clamps angles between a given min and max
 * range before calling position on hal::servo
 */
class clamped_servo : public hal::servo
{
public:
  clamped_servo(hal::strong_ptr<hal::servo> p_servo,
                hal::basic_servo::position_range_t p_range);

private:
  hal::strong_ptr<hal::servo> m_servo;
  hal::basic_servo::position_range_t m_range;
  void driver_position(hal::degrees p_position) override;
};

}  // namespace sjsu::drivers
