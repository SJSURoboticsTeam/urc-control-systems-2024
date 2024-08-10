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

#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

// #include <implementations/mission_control.hpp>
#include <implementations/pump_manager.hpp>
#include <implementations/revolver.hpp>


namespace sjsu::science {
struct hardware_map_t
{
  pump_manager* pump_controller;

  hal::servo* mixing_servo;
  revolver* revolver_controller;

  hal::steady_clock* steady_clock;
  hal::serial* terminal;
  // mission_control* mc;
  // hal::can* can;
  // hal::i2c* i2c;
  hal::callback<void()> reset;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
hardware_map_t initialize_platform();
void application(hardware_map_t& p_framework);

}  // namespace sjsu::science
