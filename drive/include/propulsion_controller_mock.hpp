#pragma once
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <propulsion_controller.hpp>
#include <velocity_motor_mock.hpp>

namespace sjsu::drive {

class propulsion_controller_mock : public propulsion_controller
{
public:
  propulsion_controller_mock(hal::v5::strong_ptr<hal::steady_clock> p_clock,
                      hal::rpm p_max_speed,
                      float p_max_acceleration);
  virtual void stop();
  virtual void set_target_velocity(hal::rpm p_velocity);
  virtual hal::rpm get_target_velocity();
  virtual hal::rpm get_actual_velocity();
private:
    drivers::velocity_motor_mock m_motor;
    hal::rpm m_target_velocity;
};
}  // namespace sjsu::drive
