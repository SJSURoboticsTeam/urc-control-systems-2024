#pragma once
#include <libhal-actuator/rc_servo.hpp>
#include <libhal/pointers.hpp>

namespace sjsu::science {

    class carousel 
    {
        public:
            carousel(hal::v5::strong_ptr<hal::actuator::rc_servo> p_servo);
            // initialize object with specified servo
            void home();
            // set servo position to 0
            void step();
            // move 1 vial clockwise (+20 degrees on servo = +40 degrees due to gear ratio)
            void step(int n);
            // move n vials clockwise (+20 degrees * n)
            void step_backwards();
            // move 1 vial counterclockwise (-20 degrees on servo = -40 degrees due to gear ratio)
            void step_backwards(int n);
            // move n vials clockwise (-20 degrees * n)
        private:
            int degrees = 0;
            hal::v5::strong_ptr<hal::actuator::rc_servo> m_carousel_servo;
    };

} // namespace sjsu::science

