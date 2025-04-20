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

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/pwm.hpp>
// #include <libhal-lpc40/system_controller.hpp> //not sure why we need this?
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include <libhal-util/units.hpp>

#include "applications/application.hpp"
namespace sjsu::drive {
void initialize_processor()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::initialize_floating_point_unit();

}

application_framework initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // Set the MCU to the maximum clock speed
  hal::lpc40::clock::maximum(12.0_MHz);

  auto& clock = hal::lpc40::clock::get();
  auto cpu_frequency = clock.get_frequency(hal::lpc40::peripheral::cpu);
  static hal::cortex_m::dwt_counter counter(cpu_frequency);

  // Serial
  static std::array<hal::byte, 1024> recieve_buffer0{};
  static auto uart0 = (hal::lpc40::uart::get(0,
                                                       recieve_buffer0,
                                                       hal::serial::settings{
                                                         .baud_rate = 38400,
                                                       }));
  // Don't think we need can for the science applications thus far
  //  hal::can::settings can_settings{ .baud_rate = 1.0_MHz };
  //  auto& can = (hal::lpc40::can::get<2>(can_settings));

  static auto in_pin0 =
    hal::lpc40::input_pin::get(1, 15, hal::input_pin::settings{});
  static auto in_pin1 =
    hal::lpc40::input_pin::get(1, 23, hal::input_pin::settings{});
  static auto in_pin2 =
    hal::lpc40::input_pin::get(1, 22, hal::input_pin::settings{});

  static auto pwm_1_6 = (hal::lpc40::pwm::get(1, 6));
  static auto pwm_1_5 = (hal::lpc40::pwm::get(1, 5));

  static auto adc_4 = hal::lpc40::adc::get(4);
  static auto adc_5 = hal::lpc40::adc::get(5);

  std::array<hal::byte, 1024> receive_buffer{};
  static auto uart1 = hal::lpc40::uart::get(0,
                                                      receive_buffer,
                                                      {
                                                        .baud_rate = 115200.0f,
                                                      });

  static auto i2c = hal::lpc40::i2c::get(2);

  return application_framework{
    .terminal = &uart0,
    // .can = &can,
    .in_pin0 = &in_pin0,
    .in_pin1 = &in_pin1,
    .in_pin2 = &in_pin2,
    .pwm_1_6 = &pwm_1_6,
    .pwm_1_5 = &pwm_1_5,
    .adc_4 = &adc_4,
    .adc_5 = &adc_5,
    .esp = &uart1,
    .i2c = &i2c,
    .steady_clock = &counter,
    .reset = []() { hal::cortex_m::reset(); },
  };
};
}  // namespace sjsu::science
