#include <libhal/motor.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>

namespace sjsu::drivers {

class velocity_motor_mock : public hal::velocity_motor
{
public:
  // max_acceleration is in rpm/s
  velocity_motor_mock(hal::v5::strong_ptr<hal::steady_clock> p_clock,
                      hal::rpm p_max_speed,
                      float p_max_acceleration);

private:
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::rpm m_max_speed;
  float m_max_acceleration;  // in rpm/s
  hal::u64 m_sim_time;

  hal::rpm m_sim_velocity = 0, m_target_velocity = 0;
  bool m_enabled = true;

  hal::rpm update_sim();

  virtual void driver_enable(bool p_state);
  virtual void driver_drive(hal::rpm p_velocity);
  virtual status_t driver_status();
  virtual range_t driver_velocity_range();
};

}  // namespace sjsu::drivers
