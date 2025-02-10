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

#pragma once

#include <array>
#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>

#include "steering_module.hpp"
#include "wheel_router.hpp"
#include "ackermann_steering.hpp"
#include "settings.hpp"



#include <optional>


namespace sjsu::drive {
  
struct hardware_map_t
{
  std::optional<hal::steady_clock*> clock;
  std::optional<hal::serial*> terminal;
  // wheel_router* router;
  // ackermann_steering* steering;
  // std::optional<hal::can*> can;
  std::optional<hal::can_transceiver*> can_transceiver;
  std::optional<hal::can_bus_manager*> can_bus_manager;
  std::optional<hal::can_identifier_filter*> can_identifier_filter;
  // std::optional<std::span<steering_module, 4>*> steering_modules;
  // std::optional<std::span<start_wheel_setting, 4>*> start_wheel_setting_span;
  std::optional<std::span<steering_module, 1>> steering_modules;
  std::optional<std::span<start_wheel_setting, 1>> start_wheel_setting_span;
  hal::callback<void()> reset;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
hardware_map_t initialize_platform();
void application(hardware_map_t& p_framework);
constexpr bool use_can_v1 = false;

}  // namespace sjsu::science
