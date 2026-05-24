#include <algorithm>
#include <cstdlib>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/serial.hpp>
#include <resource_list.hpp>
#include <sys/_intsup.h>

namespace sjsu::mimic {

void stopMotor() {
  resources::cipo1_pwm_channel()->dutycycle(0.0f);
  resources::copi1_pwm_channel()->dutycycle(0.0f);
}

void movetoPercent(float n_targetPercent) {
  n_targetPercent = std::clamp(n_targetPercent, 0.0f, 100.0f);
  
  float current_percent = resources::a0_feedback_adc()->read().value() * 100.0f;

  while (std::abs(current_percent - n_targetPercent) > tolerance) {
    current_percent = resources::a0_feedback_adc()->read().value() * 100.0f ;

    if(current_percent > n_targetPercent){
      resources::cipo1_pwm_channel()->dutycycle(motorSpeed);
      resources::copi1_pwm_channel()->dutycycle(0.0f);
    }
    else {
      resources::cipo1_pwm_channel()->dutycycle(0.0f);
      resources::copi1_pwm_channel()->dutycycle(motorSpeed);
    }  
  }
  stopMotor();
}

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();

  // pwm
  auto test_servo = resources::rc_servo();
  auto a0 = resources::a0_feedback_adc();
  auto a1 = resources::a1_adc();


  // --- Movement settings ---
  const float motorSpeed = 220.0f/225.0f;
  const int tolerance = 2 ;

  // --- Global state tracking variables ---
  float targetPercent = 0;

  hal::print(*console, "System Initialized: Proportional 0-100% Mapping Active\n");

  while(true){
    if (hal::v5::serial.available() > 0) {
      int input = hal::v5::serial.parseint();
      while (hal::v5::serial.available()) {
        hal::v5::serial.read();
      }

      if (input >= 0 && input <= 100) {
        targetPercent = input;
        hal::v5::serial.print("New serial target received : ");
        hal::v5::serial.print(targetPercent);
        hal::v5::serial.print("%");           
      }          
    }
    movetoPercent(targetPercent);
  }
}
}  // namespace sjsu::mimic