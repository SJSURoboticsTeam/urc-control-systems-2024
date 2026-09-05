#pragma once
#include "perseus_bldc.hpp"
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <steer_controller.hpp>
#include <velocity_servo_mock.hpp>

namespace sjsu::drive {

class steer_controller_perseus : public steer_controller
{
public:
  steer_controller_perseus(hal::v5::strong_ptr<drivers::perseus_bldc> p_perseus, hal::v5::strong_ptr<hal::steady_clock> p_clock);

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
  hal::v5::strong_ptr<drivers::perseus_bldc> m_perseus;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::degrees m_target_position;
  bool m_is_homed = false;
};
}  // namespace sjsu::drive
