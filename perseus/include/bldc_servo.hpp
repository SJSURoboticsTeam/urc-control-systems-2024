#include <h_bridge.hpp>
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
namespace sjsu::perseus {
// this is the perseus class itself.
class bldc_perseus
{

  // takes in quadrature encoder, pointer hbdridge
  // keeps track of status,
  // design decision: leave the can out. Main application takes care of that.
public:
  bldc_perseus(hal::v5::strong_ptr<drivers::h_bridge> p_hbridge,
               hal::v5::strong_ptr<hal::rotation_sensor> p_encoder);

  struct status  // this struct will be both target and current
  {
    hal::u16 position;
    hal::u16 velocity;
  };
  struct PID_settings
  {
    float kp = 0.1;
    float ki = 0.1;
    float kd = 0.1;
  };

  void set_target_position(hal::u16 target_position);
  hal::u16 get_target_position();
  hal::u16 get_current_position();
  // when we change velocity we must
  void set_target_velocity(hal::u16 target_velocity);
  hal::u16 get_current_velocity();
  hal::u16 get_target_velocity();
  void reset_encoder();  // this needs to happen when homed
  
private:
  status m_current;
  status m_target;
  hal::v5::strong_ptr<drivers::h_bridge>
    h_bridge;  // idk if this can be copied trivially.
  hal::v5::strong_ptr<hal::rotation_sensor>
    m_encoder;            // idk if these are supposed to be pointers or what
  float m_clamped_speed;  // encoder will never go faster than this
  float m_clamped_accel;
  float current_encoder_value;
  bool current_direction = true;  // increaeses initially.
  // if current direction changes then we know we overflowed
  // if speed is positive direction should be up and if speed is negative then
  // direction should be down
  
};

}  // namespace sjsu::perseus