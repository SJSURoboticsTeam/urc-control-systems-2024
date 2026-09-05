#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <propulsion_controller.hpp>
namespace sjsu::drive {

class propulsion_controller_rmd_x7 : public propulsion_controller
{
public:
  propulsion_controller_rmd_x7(hal::v5::strong_ptr<hal::actuator::rmd_drc_v2> p_motor);
  virtual void stop();
  virtual void set_target_velocity(hal::rpm p_velocity);
  virtual hal::rpm get_target_velocity();
  virtual hal::rpm get_actual_velocity();
private:
  hal::v5::strong_ptr<hal::actuator::rmd_drc_v2> m_motor;
  hal::rpm m_target_velocity;
};
}  // namespace sjsu::drive
