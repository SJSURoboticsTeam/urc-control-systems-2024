#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <libhal-actuator/rc_servo.hpp>

#include "../resource_list.hpp"
#include "../include/carousel.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::science {
    void application(){
        auto clock = resources::clock();
        auto terminal = resources::console();

        try{
            auto carousel_servo_ptr = resources::m_carousel_servo();
            carousel carousel_servo_one(carousel_servo_ptr);

            carousel_servo_one.step(5);
            hal::print(*terminal, "Rotated 5 vials clockwise\n");
            hal::delay(*clock, 100ms);

            carousel_servo_one.step(3);
            hal::print(*terminal, "Rotated 3 vials counterclockwise\n");
            hal::delay(*clock, 100ms);

            carousel_servo_one.home();
            hal::print(*terminal, "Returned to home vial position\n");
            hal::delay(*clock, 100ms);
        }catch(hal::exception const& e ){
            hal::print<128>(*terminal, "error code: %d\n", e.error_code());
        }catch(std::exception const& e ){
            hal::print<128>(*terminal, "error code: %s\n", e.what());
        }

    }    
}