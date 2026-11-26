#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <hr4988.hpp>
#include <resource_list.hpp>

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{
  auto clock = resources::clock();
  auto terminal = resources::console();

  auto dir_pin = resources::output_pin_3();        // ATP G3 (Pin 46 / PB4)
  auto step_pin = resources::output_pin_4();       // ATP G4 (Pin 48 / PB12)
  auto enable_pin = resources::spi_chip_select();  // ATP CS (Pin 55 / PA4)
  auto ms1_pin = resources::output_pin_0();        // ATP G0 (Pin 40 / PA0)
  auto ms2_pin = resources::output_pin_1();        // ATP G1 (Pin 42 / PA15)
  auto ms3_pin = resources::output_pin_2();        // ATP G2 (Pin 44 / PB3)

  int const full_steps_per_rotation = 200;

  enable_pin->level(true);
  hal::print(*terminal, "Starting HR4988 motor test...\n");
  hal::delay(*clock, 3s);

  hr4988 motor(dir_pin,
               step_pin,
               enable_pin,
               clock,
               hr4988::step_factor::one,
               full_steps_per_rotation,
               5ms,
               { ms1_pin, ms2_pin, ms3_pin },
               { .min = -180.0_deg, .max = 180.0_deg });

  while (true) {
    hal::print(*terminal, "Moving to 180 degrees\n");
    motor.position(180.0_deg);
    hal::delay(*clock, 1s);

    hal::print(*terminal, "Returning to zero\n");
    motor.position(0.0_deg);
    hal::delay(*clock, 1s);

    hal::print(*terminal, "Moving to -180 degrees\n");
    motor.position(-180.0_deg);
    hal::delay(*clock, 1s);

    hal::print(*terminal, "Returning to zero\n");
    motor.position(0.0_deg);
    hal::delay(*clock, 1s);
  }
}

}  // namespace sjsu::drivers
