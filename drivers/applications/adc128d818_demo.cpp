#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>

#include "../hardware_map.hpp"
#include "../include/adc128d818.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void application()
{
    auto i2c2 = resources::i2c();
    auto clock = resources::clock();
    auto terminal = resources::console();

    auto adc = drivers::adc128d818(i2c2, clock, terminal);

    while (true){
        //testing read_voltage() -reading from vADC(in0)
        auto voltage = adc.read_voltage();
        hal::print<64>(*terminal, "reading voltage: %.3f V\n", voltage);
        hal::delay(*clock, 500ms);
    }
}
}  // namespace sjsu::hub