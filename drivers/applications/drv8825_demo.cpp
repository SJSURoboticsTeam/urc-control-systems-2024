#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../include/drv8825.hpp"
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::drivers {

void application(application_framework& p_framework)
{
  // configure drivers
  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;
  
  auto m_drv8825 = drv8825(
            *p_framework.out_pin3, 
            *p_framework.out_pin4, 
            *p_framework.steady_clock, 
            drv8825::step_factor::one, 
            2048, 
            {p_framework.out_pin0,p_framework.out_pin1,p_framework.out_pin2}
        );
  
    hal::print<64>(terminal, "starting motor\n");

    while (true){
        m_drv8825.step(2048);
        hal::delay(clock, 1s);
        m_drv8825.step(-2048);
        hal::delay(clock, 1s);
    }
}
}  // namespace sjsu::drivers