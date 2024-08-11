#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include "../../platform-implementations/co2_sensor_sc40.cpp"
#include "../../platform-implementations/esp8266_mission_control.cpp"
#include "../hardware_map.hpp"


using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::science {

    hal::status application(application_framework& p_framework)
    {
       
       auto& clock = *p_framework.steady_clock;
       auto& terminal = *p_framework.terminal;
       auto& mission_control = *(p_framework.mc);
       mission_control::mc_commands commands;

        while(true){
            auto timeout = hal::create_timeout(clock, 2s);
            commands = mission_control.get_command(timeout).value();
            commands.print(&terminal); 
        }
    }
}