#include <libhal-exceptions/control.hpp>
#include <libhal-expander/pca9685.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();

  hal::print(*console, "Initializing PCA...\n\n");

  hal::expander::pca9685 pca9685(*i2c, 0b100'0000);
  auto pwm0 = pca9685.get_pwm_channel<0>();
  auto pwm1 = pca9685.get_pwm_channel<1>();

  hal::actuator::rc_servo::settings rc_servo_settings{
    .frequency = 50,
    .min_angle = 0,
    .max_angle = 180,
    .min_microseconds = 500,
    .max_microseconds = 2500,
  };

  hal::actuator::rc_servo servo_pwm0(pwm0, rc_servo_settings);
  hal::actuator::rc_servo servo_pwm1(pwm1, rc_servo_settings);

  hal::print(*console, "[pca9685] Application Starting...\n\n");

  while (true) {
    for (float deg = 145; deg >= 35; deg -= 10.0f) {
      servo_pwm0.position(deg);
      servo_pwm1.position(deg);
      hal::delay(*clock, 200ms);
    }
  }
}
}  // namespace sjsu::mimic
