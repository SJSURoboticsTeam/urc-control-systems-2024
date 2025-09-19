#include <libhal-lpc40/output_pin.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/steady_clock.hpp>

#include "../hardware_map.hpp"
#include "../include/sk9822.hpp"
#include <array>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {
struct effect_hardware
{
  light_strip_view lights;
  sk9822* driver;
  hal::v5::strong_ptr<hal::steady_clock> clock;
};

/**
 * @brief WEE WOO WEE WOO NYPD!
 *
 * it just flashes the light.
 *
 * @param hardware
 * @param on_value
 * @param off_value
 * @param period
 */
void beedoo_beedoo_beedoo(effect_hardware hardware,
                          rgb_brightness on_value,
                          rgb_brightness off_value,
                          hal::time_duration period)
{
  hal::time_duration half_period = period / 2;
  while (true) {
    light_strip_util::set_all(hardware.lights, on_value);
    hardware.driver->update(hardware.lights);
    hal::delay(*(hardware.clock), half_period);
    light_strip_util::set_all(hardware.lights, off_value);
    hardware.driver->update(hardware.lights);
    hal::delay(*(hardware.clock), half_period);
  }
}

/**
 * @brief Each led should each turn on one after each other and then turn off
 * one after each other
 *
 * @param hardware
 */
void rampup_rampdown(effect_hardware hardware)
{
  while (true) {
    for (auto i = hardware.lights.begin(); i != hardware.lights.end(); i++) {
      *i = colors::WHITE;
      hardware.driver->update(hardware.lights);
      hal::delay(*hardware.clock, 10ms);
    }
    for (auto i = hardware.lights.rbegin(); i != hardware.lights.rend(); i++) {
      *i = colors::BLACK;
      hardware.driver->update(hardware.lights);
      hal::delay(*hardware.clock, 10ms);
    }
  }
}

void application()
{
  using namespace std::literals;

  auto clock = resources::clock();
  //   auto& console = *p_resources.terminal;

  auto clock_pin = resources::output_pin_5();
  auto data_pin = resources::output_pin_6();

  light_strip<35> lights;
  sk9822 driver(clock_pin, data_pin, clock);
  light_strip_util::set_all(lights, colors::BLACK);

  rgb_brightness ON, OFF;
  ON.set(0xff, 0x00, 0x00, 0b11111);
  OFF.set(0, 0, 0, 0);

  effect_hardware hardware{ .lights = lights,
                            .driver = &driver,
                            .clock = clock };

  // beedoo_beedoo_beedoo(hardware, hal::color::red, hal::color::black, 100ms);
  rampup_rampdown(hardware);

  while (true) {
    // Print message
    // hal::print(console, "Hello, World\n");

    // for(int i = 0; i  < 35; i ++ ) {
    //   lights[i].set(0xff, 0xff, 0xff, 0b111);
    //   lights.update();
    //   hal::delay(clock, 10ms);
    //   hal::print(console, ".");
    // }
    // for(int i = 34; i >= 0; i -- ) {
    //   lights[i].set(0x00,0x00,0x00, 0b000);
    //   lights.update();
    //   hal::delay(clock, 10ms);
    //   hal::print(console, ".");
    // }
    // hal::print(console, "\n");
  }
}
};  // namespace sjsu::drivers