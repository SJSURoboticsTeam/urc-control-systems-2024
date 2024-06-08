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

#include <libhal-rmd/mc_x.hpp>
#include <libhal-soft/rc_servo.hpp>
#include <libhal-pca/pca9685.hpp>

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/pwm.hpp>
//#include <libhal-lpc40/system_controller.hpp> //not sure why we need this?
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include <libhal-util/units.hpp>
#include "../platform-implementations/esp8266_mission_control.cpp"
#include "../platform-implementations/pump_manager.hpp"
#include "../platform-implementations/revolver.hpp"
#include "../platform-implementations/helper.hpp"

#include "../applications/application.hpp"

namespace sjsu::science {
hal::status initialize_processor()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::initialize_floating_point_unit();

  return hal::success();
}

hal::result<application_framework> initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // Set the MCU to the maximum clock speed
  // HAL_CHECK(hal::lpc40::clock::maximum(12.0_MHz));

  // auto& clock = hal::lpc40::clock::get();
  // auto cpu_frequency = clock.get_frequency(hal::lpc40::peripheral::cpu);
  // static hal::cortex_m::dwt_counter counter(cpu_frequency);

   // setting clock
  HAL_CHECK(hal::lpc40::clock::maximum(12.0_MHz));
  auto& clock = hal::lpc40::clock::get();
  // static auto i2c1 = HAL_CHECK((hal::lpc40::i2c::get(1,
  //                                                    hal::i2c::settings{
  //                                                      .clock_rate = 100.0_kHz,
  //                                                    })));
  // static auto i2c2 = HAL_CHECK((hal::lpc40::i2c::get(2,
  //                                                    hal::i2c::settings{
  //                                                      .clock_rate = 100.0_kHz,
  //                                                    })));
  auto cpu_frequency = clock.get_frequency(hal::lpc40::peripheral::cpu);
  static hal::cortex_m::dwt_counter counter(cpu_frequency);

  // Serial
  static std::array<hal::byte, 1024> recieve_buffer0{};
  static auto uart0 = HAL_CHECK((hal::lpc40::uart::get(0,
                                                       recieve_buffer0,
                                                       hal::serial::settings{
                                                         .baud_rate = 38400,
                                                       })));

  static std::array<hal::byte, 8192> recieve_buffer1{};

  static auto uart1 = HAL_CHECK((hal::lpc40::uart::get(1,
                                                       recieve_buffer1,
                                                       hal::serial::settings{
                                                         .baud_rate = 115200,
                                                       })));

  static constexpr std::string_view ssid =
    "TP-Link_FC30";  // change to wifi name that you are using
  static constexpr std::string_view password =
    "R0Bot1cs3250";  // change to wifi password you are using

  // still need to decide what we want the static IP to be
  static constexpr std::string_view ip = "192.168.0.216";
  static constexpr auto socket_config = hal::esp8266::at::socket_config{
    .type = hal::esp8266::at::socket_type::tcp,
    .domain = "192.168.0.211",
    .port = 5000,
  };
  HAL_CHECK(hal::write(uart0, "created Socket\n"));
  // static constexpr char* get_request = "GET / %s HTTP/1.1\r\n"
  //                                                 "Host: 192.168.0.211:5000\r\n"
  //                                                 "Keep-Alive: timeout=1000\r\n"
  //                                                 "Connection: keep-alive\r\n"
  //                                                 "\r\n";
  
  // static constexpr std::string_view get_request_meta_data = "";


  // char get_request[get_prefix.size() + get_request_meta_data.size() + 200];
  static std::array<hal::byte, 2048> buffer{};
  // static auto helper = serial_mirror(uart1, uart0);

  auto timeout = hal::create_timeout(counter, 10s);
  //break right here
  static auto esp8266 = HAL_CHECK(hal::esp8266::at::create(uart1, timeout));
  HAL_CHECK(hal::write(uart0, "created AT\n"));

  auto mc_timeout = hal::create_timeout(counter, 10s);
  static auto esp_mission_control =
    sjsu::science::esp8266_mission_control::create(esp8266,
                                               uart0,
                                               ssid,
                                               password,
                                               socket_config,
                                               ip,
                                               mc_timeout,
                                               buffer);
  while (esp_mission_control.has_error()) {
    mc_timeout = hal::create_timeout(counter, 10s);
    esp_mission_control =
      sjsu::science::esp8266_mission_control::create(esp8266,
                                                 uart0,
                                                 ssid,
                                                 password,
                                                 socket_config,
                                                 ip,
                                                 mc_timeout,
                                                 buffer);
  }
  static auto science_mission_control = esp_mission_control.value();

  static hal::can::settings can_settings{ .baud_rate = 1.0_MHz };
  static auto can = HAL_CHECK((hal::lpc40::can::get(2, can_settings)));

  static auto can_router = hal::can_router::create(can).value();

  // static auto mixing_mc_x =
  //   HAL_CHECK(hal::rmd::mc_x::create(can_router, counter, 36.0, 0x141));
  // static auto mixing_servo =
  //   HAL_CHECK(hal::make_servo(mixing_mc_x ,2.0_rpm));

  // Don't think we need can for the science applications thus far
  //  hal::can::settings can_settings{ .baud_rate = 1.0_MHz };
  //  auto& can = HAL_CHECK((hal::lpc40::can::get<2>(can_settings)));

  // static auto pca9685 = HAL_CHECK(hal::pca::pca9685::create(i2c,0b100'0000)); 
  // static auto pwm0 = pca9685.get_pwm_channel<3>(); 
  // static auto revolver_settings = hal::soft::rc_servo::settings
  //   {
  //     .min_angle = hal::degrees(0.0),
  //     .max_angle = hal::degrees(180.0),
  //     .min_microseconds = 1000,
  //     .max_microseconds = 2000,
  //   };

  //   static auto revolver_servo =
  //     HAL_CHECK(hal::soft::rc_servo::create(pwm0, rotunda_science_settings));
    
  //   static auto in_pin2 =
  //     HAL_CHECK(hal::lpc40::input_pin::get(1, 22));

  //   static auto revolver = HAL_CHECK(sjsu::science::revolver::create(revolver_servo, in_pin2, steady_clock));

  static auto in_deionized_water_pump_pin =
    HAL_CHECK(hal::lpc40::output_pin::get(1, 19, hal::output_pin::settings{}));
  static auto in_sample_pump_pin =
    HAL_CHECK(hal::lpc40::output_pin::get(1, 23, hal::output_pin::settings{}));
  static auto in_molisch_pump_pin =
    HAL_CHECK(hal::lpc40::output_pin::get(1, 22, hal::output_pin::settings{}));
  static auto in_sulfuric_acid_pin =
    HAL_CHECK(hal::lpc40::output_pin::get(1, 21, hal::output_pin::settings{}));
  static auto in_biuret_pump_pin =
    HAL_CHECK(hal::lpc40::output_pin::get(1, 20, hal::output_pin::settings{}));

  static auto pump_controller = HAL_CHECK(pump_manager::create(
    counter, 
    in_deionized_water_pump_pin,
    in_sample_pump_pin, 
    in_molisch_pump_pin,
    in_sulfuric_acid_pin,
    in_biuret_pump_pin));
    
//Mixing servo
  static auto i2c = HAL_CHECK(hal::lpc40::i2c::get(2));
  static auto pca9685 = HAL_CHECK(hal::pca::pca9685::create(i2c, 0b100'0000)); 
  static auto pwm0 = pca9685.get_pwm_channel<0>(); 
  auto servo_settings = hal::soft::rc_servo::settings{
    .min_angle = 0.0_deg,
    .max_angle = 360.0_deg,
    .min_microseconds = 500,
    .max_microseconds = 2500,
  }; 

  static auto mixing_servo = HAL_CHECK(hal::soft::rc_servo::create(pwm0, servo_settings));

//Creating Revolver
  static auto i2c_2 = HAL_CHECK(hal::lpc40::i2c::get(3));
  static auto pca9685_2 = HAL_CHECK(hal::pca::pca9685::create(i2c_2, 0b100'0000)); 
  static auto pwm1 = pca9685_2.get_pwm_channel<1>(); 
  auto revolver_servo_settings = hal::soft::rc_servo::settings{
      .min_angle = hal::degrees(0.0),
      .max_angle = hal::degrees(180.0),
      .min_microseconds = 1000,
      .max_microseconds = 2000,
    };
   static auto revolving_servo = HAL_CHECK(hal::soft::rc_servo::create(pwm1, revolver_servo_settings));

  static auto revolver_pin =
    HAL_CHECK(hal::lpc40::input_pin::get(1, 15, hal::input_pin::settings{}));

  static auto revolver_controller = HAL_CHECK(revolver::create(revolving_servo, revolver_pin, counter, uart0));


  // static auto pwm_1_6 = HAL_CHECK((hal::lpc40::pwm::get(1, 6)));
  // static auto pwm_1_5 = HAL_CHECK((hal::lpc40::pwm::get(1, 5)));

  // static auto adc_4 = HAL_CHECK(hal::lpc40::adc::get(4));
  // static auto adc_5 = HAL_CHECK(hal::lpc40::adc::get(5));

  // std::array<hal::byte, 1024> recei4ve_buffer{};

  // static auto i2c = HAL_CHECK(hal::lpc40::i2c::get(2));
  // static auto can = HAL_CHECK(hal::lpc40::can::get(0));
  
  return application_framework{
    
    // .in_deionized_water_pump_pin = &in_deionized_water_pump_pin,
    // .in_sample_pump_pin= &in_sample_pump_pin,
    // .in_molisch_pump_pin= &in_molisch_pump_pin,
    // .in_sulfuric_acid_pin= &in_sulfuric_acid_pin,
    // .in_biuret_pump_pin= &in_biuret_pump_pin,
    .pump_controller = &pump_controller,
    // .in_pin1 = &in_pin1,
    // .in_pin2 = &in_pin2, //hall effect sensor
    // .pwm_1_6 = &pwm_1_6,
    // .pwm_1_5 = &pwm_1_5,
    // .adc_4 = &adc_4,
    // .adc_5 = &adc_5,
    .mixing_servo = &mixing_servo,
    .revolver_controller = &revolver_controller,

    // .revolver_servo = &revolver_servo,
    .steady_clock = &counter,
    // .revolver = &revolver,
    .terminal = &uart0, //uart0 is terminal
    .mc = &science_mission_control,
    // .can = &can,
    // .i2c = &i2c,
    .reset = []() { hal::cortex_m::reset(); },
  }; 
};
}  // namespace sjsu::science
