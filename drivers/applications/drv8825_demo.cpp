#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <drv8825.hpp>
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  // configure drivers
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;

    drv8825 stepper_controller(drv8825::ctor_params{
            .direction_pin = *p_framework.out_pin3,
            .step_pin = *p_framework.out_pin4,
            .steady_clock = *p_framework.steady_clock,
            .motor_step_factor = drv8825::step_factor::one, 
            .steps_per_rotation = 2048, 
            .step_half_period = 450us,
            .mode_pins = {p_framework.out_pin0,p_framework.out_pin1,p_framework.out_pin2}
        });

    hal::print(terminal, "starting motor\n");

    while (true){
        stepper_controller.step(2048);
        hal::delay(clock, 1s);
        stepper_controller.step(-2048);
        hal::delay(clock, 1s);
    }
}
}  // namespace sjsu::drivers