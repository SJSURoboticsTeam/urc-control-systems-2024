#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <can_messaging.hpp>
#include <switches.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {


void application()
{
    using namespace hal::literals;
    using namespace std::chrono_literals;

    auto clock = resources::clock(); 
    auto console = resources::console(); 

    auto switches = switches_bldc(resources::switch_g1(), 
                    resources::switch_g2(), 
                    resources::switch_g3(), 
                    resources::switch_g4(), 
                    resources::switch_g5(), 
                    resources::switch_g6()); 
    auto switches_ptr = hal::v5::make_strong_ptr<decltype(switches)>(resources::driver_allocator(), std::move(switches));

    while(true) {
    
        hal::print<64>(*console, "Pins: %d\n", switches_ptr->read_switch_value()); 
        hal::delay(*clock, 5000ms); 
    
    }

}
}  // namespace sjsu::perseus