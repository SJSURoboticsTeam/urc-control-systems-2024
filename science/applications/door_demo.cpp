#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <libhal-actuator/rc_servo.hpp>

#include "../resource_list.hpp"

using namespace hal::literals;
using namespace std::chrono_literals;

namespace sjsu::science {
    void application() {
        auto clock = resources::clock();
        auto terminal = resources::console();
        auto door_servo_ptr = resources::door_servo();
        auto top_door_limit_switch_ptr = resources::top_door_limit_switch();
        auto bottom_door_limit_switch_ptr = resources::bottom_door_limit_switch();

        /*
        // test if top door limit switch works
        while (true) {
            if (top_door_limit_switch_ptr->level()) {
                hal::print(*terminal, "Top limit switch is pressed\n");
                break;
            }
        }
        */

        /*
        // test if checking if top door limit switch is not pressed works
        while (true) {
            if (!top_door_limit_switch_ptr->level()) {
                hal::print(*terminal, "Top limit switch is not pressed\n");
                break;
            }
        }
        */

        /*
        // test if bottom door limit switch works
        while (true) {
            if (bottom_door_limit_switch_ptr->level()) {
                hal::print(*terminal, "Bottom limit switch is pressed\n");
                break;
            }
        }
        */

        // BEFORE TESTING MOVE DOOR TO HALFWAY POINT (AVOID BREAKING LIMIT SWITCHES)

        /*
        // test moving door up 
        try{
            door_servo_ptr->position(45); // ~1/2 max speed ccw
            hal::delay(*clock, 2000ms);
            door_servo_ptr->position(90); // stop servo
            hal::delay(*clock, 1000ms);
        }
        catch(const char* err) {
            hal::print<16>(*terminal, "%s\n", err);
        }
        hal::print(*terminal, "Spun servo CCW at ~1/2 max speed\n");
        hal::delay(*clock, 100ms);
        */

        /*
        // test moving door down 
        try{
            door_servo_ptr->position(135); // ~1/2 max speed cw
            hal::delay(*clock, 2000ms);
            door_servo_ptr->position(90); // stop servo
            hal::delay(*clock, 1000ms);
        }
        catch(const char* err) {
            hal::print<16>(*terminal, "%s\n", err);
        }
        hal::print(*terminal, "Spun servo CW at ~1/2 max speed\n");
        hal::delay(*clock, 100ms);
        */

        /*
        // open door completely
        try{
            if (!top_door_limit_switch_ptr->level()) {
            // check if door is open before attempting to move servo (in order to not break limit switch)
                door_servo_ptr->position(45); // ~1/2 max speed ccw
                while (true) {  
                    if (top_door_limit_switch_ptr->level()) {
                        door_servo_ptr->position(90); // stop servo when top limit switch is pressed
                        hal::delay(*clock, 1000ms);
                        break;
                    }
                }
            }
        }
        catch(const char* err) {
            hal::print<16>(*terminal, "%s\n", err);
        }
        hal::print(*terminal, "Door is open\n");
        hal::delay(*clock, 100ms);
        */

        
        // close door completely
        try{
            if (!bottom_door_limit_switch_ptr->level()) {
            // check if door is open before attempting to move servo (in order to not break limit switch)
                door_servo_ptr->position(135); // ~1/2 max speed cw
                while (true) {  
                    if (bottom_door_limit_switch_ptr->level()) {
                        door_servo_ptr->position(90); // stop servo when bottom limit switch is pressed
                        hal::delay(*clock, 1000ms);
                        break;
                    }
                }
            }
        }
        catch(const char* err) {
            hal::print<16>(*terminal, "%s\n", err);
        }
        hal::print(*terminal, "Door is closed\n");
        hal::delay(*clock, 100ms);
        
    }
}