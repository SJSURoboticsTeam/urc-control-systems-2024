#pragma once
#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

class velocity_servo_mock : public hal::velocity_servo
{
public:
  velocity_servo_mock(hal::v5::strong_ptr<hal::steady_clock> p_clock,
                      hal::rpm p_max_speed,
                      position_range_t p_position_range,
                      hal::degrees p_inital_position);

private:
  bool m_enabled = true;
  hal::degrees m_sim_position;
  hal::u64 m_sim_time;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::rpm m_max_speed;
  hal::degrees m_target_position;
  position_range_t m_postion_range;

  hal::degrees update_sim();

  virtual void driver_enable(bool p_state);
  virtual void driver_position(hal::degrees p_target_position);
  virtual position_range_t driver_position_range();
  virtual hal::degrees driver_get_position();
  virtual bool driver_is_moving();
  virtual void driver_configure(settings const& p_settings);
  virtual status_t driver_status();
  virtual range_t driver_velocity_range();
};
};  // namespace sjsu::drivers
