// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-soft/rc_servo.hpp>
#include <libhal-pca/pca9685.hpp>

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-pca/pca9685.hpp>
#include <libhal-lpc40/pwm.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include <libhal-util/units.hpp>

// #include <implementations/esp8266_mission_control.cpp>
#include "../include/pump_manager.hpp"
#include "../include/revolver.hpp"
// #include <implementations/helper.hpp>

#include <libhal-micromod/micromod.hpp>
#include <applications/application.hpp>

namespace sjsu::science {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  auto& counter = hal::micromod::v1::uptime_clock();

  auto& output_deionized_water_pump_pin = hal::micromod::v1::output_g0();
  auto& output_sample_pump_pin = hal::micromod::v1::output_g1();
  auto& output_molisch_pump_pin = hal::micromod::v1::output_g2();
  auto& output_sulfuric_acid_pin = hal::micromod::v1::output_g3();
  auto& output_biuret_pump_pin = hal::micromod::v1::output_g4();

  static pump_manager pump_controller(
    counter, 
    output_deionized_water_pump_pin,
    output_sample_pump_pin, 
    output_molisch_pump_pin,
    output_sulfuric_acid_pin,
    output_biuret_pump_pin);
    
//Mixing servo
  auto& i2c = hal::micromod::v1::i2c();
  static constexpr hal::byte address = 0b100'0000;
  hal::pca::pca9685 pca9685(i2c, address);

  auto pwm0 = pca9685.get_pwm_channel<0>(); 
  auto servo_settings = hal::soft::rc_servo::settings{
    .min_angle = 0.0_deg,
    .max_angle = 360.0_deg,
    .min_microseconds = 500,
    .max_microseconds = 2500,
  };

  static hal::soft::rc_servo mixing_servo(pwm0, servo_settings);

//Creating Revolver
  static auto pwm1 = pca9685.get_pwm_channel<1>(); 
  auto revolver_servo_settings = hal::soft::rc_servo::settings{
      .min_angle = hal::degrees(0.0),
      .max_angle = hal::degrees(180.0),
      .min_microseconds = 1000,
      .max_microseconds = 2000,
    };
  static hal::soft::rc_servo revolving_servo(pwm1, revolver_servo_settings);

  auto& revolver_pin =  hal::micromod::v1::input_g0();

  auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

  static revolver revolver_controller(revolving_servo, revolver_pin, counter, terminal);

  
  return hardware_map_t{
    
    .pump_controller = &pump_controller,
    .mixing_servo = &mixing_servo,
    .revolver_controller = &revolver_controller,

    .steady_clock = &counter,
    .terminal = &terminal,
    
    .deionized_water_pump = &hal::micromod::v1::output_g0(),
    .sample_pump = &hal::micromod::v1::output_g1(),
    .molisch_reagent_pump = &hal::micromod::v1::output_g2(),
    .sulfuric_acid_pump = &hal::micromod::v1::output_g3(),
    .biuret_reagent = &hal::micromod::v1::output_g4(),
    
    // .mc = test_mc,
    .reset = []() { hal::cortex_m::reset(); },
  }; 
};
}  // namespace sjsu::science
