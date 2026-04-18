

// rc servo x 8
// analog
// clock
// serial

// 4 servos
//      make adc
//      
// 1 dc motor/encoder
// lights


// class to take in adc and rc servo
// read and write to feedback_servo
//      start record funxtion - an array to be recorded?

// copy conan, cmakelists, and main.cpp for things to work and hardwaremap.hpp
//      refer to hub

// main file is application

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <resource_list.hpp>

namespace sjsu::mimic {
void application()
{
  using namespace std::chrono_literals;

  // auto& led = *hardware_map.led;
  auto led = resources::status_led();
  auto clock = resources::clock();
  auto console = resources::console();
 
  // pwm
  // auto test_servo = resources::rc_servo();
  auto test_pwm = resources::test_servo_pwm_channel_0();
  auto test_servo_feedback = resources::test_servo_feedback_adc_0();

  hal::print(*console, "Starting Application!\n");
  hal::print(*console, "Will reset after ~10 seconds\n");

  while (true) {
    for (uint16_t i = 0x00ff; i < 0xff00; i+=8) {
      test_pwm->duty_cycle(i);
      hal::delay(*clock, 1ms);
    }


    // hal::print(*console, "Set to: -80 deg\n");
    // test_servo->position(-80.0);
    // hal::delay(*clock, 2000ms);
    // hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());

    // hal::print(*console, "Set to: 0 deg\n");
    // test_servo->position(0.0);
    // hal::delay(*clock, 2000ms);
    // hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());

    // hal::print(*console, "Set to: 80 deg\n");
    // test_servo->position(180.0);
    // hal::delay(*clock, 2000ms);
    // hal::print<64>(*console, "Test servo position: %f\n", test_servo_feedback->read());
  }

  // currently tests adafruit feedback servo
  // next, test motorized linear pot
  // array of servos? so add PCA
  // add ADC1283

  // skeleton
  /*
  initialize array of servos?
  something something PCA PWM channels
  initialize ADC1283 ADC channels
  initialize usb serial

  Buttons
  mode == mimic
    read from serial current arm positions
    set through PCA PWM

  mode == controller
    disable power? signal? (is this me or ee)
    do we want button to only unpower that specific joint or the whole thing
    write joint info and degree through serial
  */
  

  // hal::print(*console, "Resetting!\n");
  // hal::delay(*clock, 100ms);
}
}  // namespace sjsu::mimic