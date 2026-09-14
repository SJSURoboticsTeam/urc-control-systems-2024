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

    auto switches_ptr = resources::switches(); 
    while(true) {
    
        hal::print<64>(*console, "Pins: %d\n", switches_ptr->read_switch_value()); 
        hal::delay(*clock, 5000ms); 
    
    }

}
}  // namespace sjsu::perseus