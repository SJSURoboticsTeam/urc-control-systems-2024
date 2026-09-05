#pragma once
#include <libhal/units.hpp>
class steer_controller {
public:
  virtual void stop() = 0;
  
  virtual void hard_home() = 0;
  virtual void home() = 0;
  virtual void home_periodic() = 0;
  virtual bool is_homing() = 0;
  virtual bool is_homed() = 0;

  virtual void set_target_position(hal::degrees p_target_position) = 0;
  virtual hal::degrees get_target_postion() = 0;
  virtual hal::degrees get_actual_postion() = 0;
  
};