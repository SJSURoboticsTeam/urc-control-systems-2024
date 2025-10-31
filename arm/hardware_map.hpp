// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include "perseus_bldc.hpp"
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/zero_copy_serial.hpp>
namespace sjsu::arm {
namespace custom {
/**
 * @brief A stand in interface until libhal supports an official watchdog
 * interface.
 *
 */
class watchdog
{
public:
  watchdog() = default;
  virtual void start() = 0;
  virtual void reset() = 0;
  virtual void set_countdown_time(hal::time_duration p_wait_time) = 0;
  virtual bool check_flag() = 0;
  virtual void clear_flag() = 0;
  virtual ~watchdog() = default;
};
}  // namespace custom
namespace resources {
// =======================================================
// Defined by each platform file
// =======================================================
/**
 * @brief Allocator for driver memory
 *
 * The expectation is that the implementation of this allocator is a
 * std::pmr::monotonic_buffer_resource with static memory storage, meaning the
 * memory is fixed in size and memory cannot be deallocated. This is fine for
 * the demos.
 *
 * @return std::pmr::polymorphic_allocator<>
 */
std::pmr::polymorphic_allocator<> driver_allocator();
/**
 * @brief Steady clock that provides the current uptime
 *
 * @return hal::v5::strong_ptr<hal::steady_clock>
 */
struct arm_can_finders
{
  hal::v5::strong_ptr<hal::can_message_finder> home_finder;
  hal::v5::strong_ptr<hal::can_message_finder> arm_finder;
  hal::v5::strong_ptr<hal::can_message_finder> endeffector_finder;
};

struct arm_joints
{
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> track_servo;
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> shoulder_servo;
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> elbow_servo;
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> wrist_pitch_servo;
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> wrist_roll_servo;
  hal::v5::strong_ptr<sjsu::drivers::perseus_bldc> clamp_servo;
};

hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::zero_copy_serial> zero_copy_serial();
hal::v5::strong_ptr<hal::output_pin> status_led();
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager();
hal::v5::strong_ptr<hal::can_interrupt> can_interrupt();

arm_can_finders can_finders(
  hal::v5::strong_ptr<hal::can_transceiver> transceiver,
  hal::u16 home,
  hal::u16 arm,
  hal::u16 endeffector);

arm_joints arm_servos(hal::v5::strong_ptr<hal::can_transceiver> transceiver);

inline void reset()
{
  hal::cortex_m::reset();
}
inline void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}
}  // namespace resources

// Application function is implemented by one of the .cpp files.
void initialize_platform();
void application();
}  // namespace sjsu::arm