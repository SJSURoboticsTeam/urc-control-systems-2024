#include <libhal/motor.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
namespace sjsu::drivers {

class h_bridge : public hal::motor
{
public:
  explicit h_bridge(hal::pwm16_channel& p_pin_a_low,
                    hal::pwm16_channel& p_pin_b_low,
                    hal::output_pin& p_pin_a_high,
                    hal::output_pin& p_pin_b_high)
    : m_pin_a_low(p_pin_a_low)
    , m_pin_b_low(p_pin_b_low)
    , m_pin_a_high(p_pin_a_high)
    , m_pin_b_high(p_pin_b_high)
  {
  }

private:
  hal::pwm16_channel& m_pin_a_low;
  hal::pwm16_channel& m_pin_b_low;
  hal::output_pin& m_pin_a_high;
  hal::output_pin& m_pin_b_high;

  void driver_power(float p_power);
};
}  // namespace sjsu::drivers
