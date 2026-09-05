#pragma once
#include <libhal/units.hpp>
#include <steer_controller.hpp>
#include <velocity_servo_mock.hpp>

namespace sjsu::drive {

class steer_controller_mock : public steer_controller
{
public:
  steer_controller_mock(hal::v5::strong_ptr<hal::steady_clock> p_clock,
                        hal::rpm p_max_speed,
                        hal::degrees p_inital_position);

  virtual void stop();
  
  virtual void hard_home();
  virtual void home();
  virtual void home_periodic();
  virtual bool is_homing();
  virtual bool is_homed();

  virtual void set_target_position(hal::degrees p_target_position);
  virtual hal::degrees get_target_postion();
  virtual hal::degrees get_actual_postion();

private:
  drivers::velocity_servo_mock m_servo;
  hal::degrees m_target_position = 0.0f;
  bool m_is_homed = false;
};
}  // namespace sjsu::drive
