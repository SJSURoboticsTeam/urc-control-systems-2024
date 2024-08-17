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
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

namespace sjsu::drivers {
struct application_framework
{
  hal::serial* terminal;
  hal::can* can;
  hal::input_pin* in_pin0;
  hal::input_pin* in_pin1;
  hal::input_pin* in_pin2;
  hal::output_pin* led;
  hal::output_pin* out_pin0;
  hal::output_pin* out_pin1;
  hal::output_pin* out_pin2;
  hal::pwm* pwm0;
  hal::pwm* pwm1;
  hal::adc* adc0;
  hal::adc* adc1;
  // hal::serial* esp;
  hal::i2c* i2c;
  hal::steady_clock* steady_clock;
  hal::callback<void()> reset;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
void initialize_processor();
application_framework initialize_platform();
void application(application_framework& p_framework);

}  // namespace sjsu::drivers
