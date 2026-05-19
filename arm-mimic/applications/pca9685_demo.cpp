#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-expander/pca9685.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();

  hal::print(*console, "Initializing PCA...\n\n");

  // pca
  hal::expander::pca9685 pca9685(*i2c, 0b100'0000);
  auto pwm0 = pca9685.get_pwm_channel<0>();
  auto pwm1 = pca9685.get_pwm_channel<1>();
  auto pwm2 = pca9685.get_pwm_channel<2>();
  auto pwm3 = pca9685.get_pwm_channel<3>();

  hal::print(*console, "[pca9685] Application Starting...\n\n");

  // Setting the frequency of one channel will set the frequency of all channels
  pwm0.frequency(50.0);

  constexpr float period_us = 20000.0f;
  while (true) {
    for (float pulse_us = 500.0f; pulse_us <= 2500.0f; pulse_us += 10.0f) {
      float duty_cycle = pulse_us / period_us;
      hal::print<64>(*console, "pulse = %f us, duty = %f\n", pulse_us, duty_cycle);

      hal::delay(*clock, 50ms);
      pwm0.duty_cycle(duty_cycle);
      pwm1.duty_cycle(duty_cycle);
      pwm2.duty_cycle(duty_cycle);
      pwm3.duty_cycle(duty_cycle);
    }
  }
}
}  // namespace sjsu::mimic