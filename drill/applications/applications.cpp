
#include <exception>
#include "../../drivers/include/drv8825.hpp"
#include "../../drivers/include/soil_sensor_sht21.hpp"
#include "implementation.hpp"
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <../resource_list.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <memory_resource>
using namespace hal::literals;
using namespace std::chrono_literals;



namespace sjsu::drill
{
  void application()
  {
    std::pmr::polymorphic_allocator<> alloc{};

    auto clock    = resources::clock();
    auto i2c      = resources::i2c();
    auto terminal = resources::console();
    auto manager = resources::can_bus_manager();
    auto transceiver = resources::can_transceiver();
    auto filter = resources::can_filter();

    auto dir_pin  = resources::output_pin_5();
    auto step_pin = resources::output_pin_0();
    auto m1_pin   = resources::output_pin_7();
    auto m2_pin   = resources::output_pin_6();
    auto m3_pin   = resources::output_pin_4();

    auto stepper_controller =
      hal::v5::make_strong_ptr<sjsu::drivers::drv8825>(
        alloc,
        sjsu::drivers::drv8825::ctor_params{
          .direction_pin = dir_pin,
          .step_pin = step_pin,
          .steady_clock = clock,
          .motor_step_factor =
            sjsu::drivers::drv8825::step_factor::one,
          .full_steps_per_rotation = 2048,
          .step_half_period = 450us,
          .mode_pins = { m1_pin, m2_pin, m3_pin },
        });

    auto soil =
      hal::v5::make_strong_ptr<sjsu::drivers::sht21>(alloc, i2c);

    auto drill =
      drill_class(
        resources::drill(),
        clock,
        stepper_controller,
        soil);

      while(true)
      {
        drill.stop();
        hal::delay(*clock, 500ms);
        drill.set_velocity(-20.0);
        hal::delay(*clock, 500ms);
        // stepper_controller.step(2400);
        
        // stepper_controller.step(2400);

      }
  }
}