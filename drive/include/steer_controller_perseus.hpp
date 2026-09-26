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
  /**
   * p_perseus should not be touched after passing it to steer_controller
   * otherwise one may run the risk of corrupting state!
   */
  steer_controller_perseus(hal::v5::strong_ptr<drivers::perseus_bldc> p_perseus, hal::v5::strong_ptr<hal::steady_clock> p_clock);

  void stop() override;

  void hard_home() override;
  void home() override;
  void home_periodic() override;
  bool is_homing() override;
  bool is_homed() override;
  void stop_home() override;

  void set_target_position(hal::degrees p_target_position) override;
  hal::degrees get_target_postion() override;
  hal::degrees get_actual_postion() override;

private:
  hal::v5::strong_ptr<drivers::perseus_bldc> m_perseus;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::degrees m_target_position;
  bool m_is_homed = false;
};
}  // namespace sjsu::drive
