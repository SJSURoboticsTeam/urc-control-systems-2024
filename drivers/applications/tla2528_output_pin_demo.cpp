#include <array>
#include <libhal-util/steady_clock.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/units.hpp>
#include <tla2528.hpp>
#include <tla2528_adapters.hpp>
#include "../hardware_map.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::drivers {

void application(application_framework& p_framework) {
    bool demo_open_drain = false;
    hal::byte i2c_address = 0x10;// 0x10 is the tla address for no resistors
    float analog_supply_voltage = 3.3;

    auto& i2c = *p_framework.i2c;
    auto& steady_clock = *p_framework.steady_clock;
    tla2528 gpo_expander = tla2528(i2c, i2c_address, analog_supply_voltage);
    std::array<tla2528_output_pin, 8> gpos {
        make_output_pin(gpo_expander,0,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,1,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,2,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,3,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,4,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,5,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,6,{ .open_drain = demo_open_drain}),
        make_output_pin(gpo_expander,7,{ .open_drain = demo_open_drain})
    };

    while (true) {
        for (int i=0; i<8; i++) {
            gpos[i].level(true);
            hal::delay(steady_clock, 250ms);
        }
        for (int i=0; i<8; i++) {
            gpos[i].level(false);
            hal::delay(steady_clock, 250ms);
        }
    }
}
}  // namespace sjsu::drivers